// Windows boids screen saver. Flocking model inspired by rystrauss/boids
// (MIT; Copyright 2020-2021 Ryan Strauss). This implementation is standalone:
// Win32, OpenGL, and fixed-step simulation.

#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#include <windowsx.h>
#include <GL/gl.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include "flock_simulation.h"
#include "settings_dialog.h"

namespace {

constexpr char kWindowClass[] = "BoidsScreensaverWindow";
constexpr char kAppTitle[] = "Boids Screensaver";
constexpr UINT_PTR kFrameTimerId = 1;
constexpr UINT kFrameTimerMilliseconds = 8;
constexpr float kSimulationStepSeconds = 1.0f / 60.0f;
constexpr float kMaximumFrameSeconds = 0.25f;
constexpr int kAdjustmentWidth = 1000;
constexpr int kAdjustmentHeight = 700;

HINSTANCE g_instance = nullptr;
HWND g_window = nullptr;
HDC g_device_context = nullptr;
HGLRC g_render_context = nullptr;
bool g_preview = false;
bool g_fullscreen = false;
bool g_adjustment_mode = false;
bool g_exit_on_input = false;
HWND g_adjustment_panel = nullptr;
POINT g_initial_mouse{};
int g_client_width = 1;
int g_client_height = 1;
std::chrono::steady_clock::time_point g_last_frame_time;
float g_accumulator = 0.0f;
boids::FlockSimulation* g_simulation = nullptr;

void message_box(const char* text, UINT flags = MB_OK | MB_ICONERROR) {
    MessageBoxA(nullptr, text, kAppTitle, flags);
}

bool parse_preview_parent(char* text, HWND* parent) {
    while (*text == ' ' || *text == '\t' || *text == ':') {
        ++text;
    }
    if (*text == '\0') {
        return false;
    }

    char* end = nullptr;
    const unsigned long long raw = std::strtoull(text, &end, 0);
    if (end == text || raw == 0) {
        return false;
    }

    *parent = reinterpret_cast<HWND>(static_cast<uintptr_t>(raw));
    return IsWindow(*parent) != FALSE;
}

void get_client_size(HWND window) {
    RECT rect{};
    GetClientRect(window, &rect);
    g_client_width = std::max(1L, rect.right - rect.left);
    g_client_height = std::max(1L, rect.bottom - rect.top);
}

bool initialize_opengl(HWND window) {
    g_device_context = GetDC(window);
    if (g_device_context == nullptr) {
        return false;
    }

    PIXELFORMATDESCRIPTOR pixel_format{};
    pixel_format.nSize = sizeof(pixel_format);
    pixel_format.nVersion = 1;
    pixel_format.dwFlags = PFD_DRAW_TO_WINDOW | PFD_SUPPORT_OPENGL | PFD_DOUBLEBUFFER;
    pixel_format.iPixelType = PFD_TYPE_RGBA;
    pixel_format.cColorBits = 32;
    pixel_format.cDepthBits = 24;
    pixel_format.iLayerType = PFD_MAIN_PLANE;

    const int format = ChoosePixelFormat(g_device_context, &pixel_format);
    if (format == 0 || !SetPixelFormat(g_device_context, format, &pixel_format)) {
        ReleaseDC(window, g_device_context);
        g_device_context = nullptr;
        return false;
    }

    g_render_context = wglCreateContext(g_device_context);
    if (g_render_context == nullptr || !wglMakeCurrent(g_device_context, g_render_context)) {
        if (g_render_context != nullptr) {
            wglDeleteContext(g_render_context);
            g_render_context = nullptr;
        }
        ReleaseDC(window, g_device_context);
        g_device_context = nullptr;
        return false;
    }

    glDisable(GL_DEPTH_TEST);
    glDisable(GL_CULL_FACE);
    glDisable(GL_BLEND);
    glClearColor(0.01f, 0.02f, 0.05f, 1.0f);
    return true;
}

void shutdown_opengl() {
    if (g_render_context != nullptr) {
        wglMakeCurrent(nullptr, nullptr);
        wglDeleteContext(g_render_context);
        g_render_context = nullptr;
    }
    if (g_device_context != nullptr && g_window != nullptr) {
        ReleaseDC(g_window, g_device_context);
        g_device_context = nullptr;
    }
}

void update_projection() {
    glViewport(0, 0, g_client_width, g_client_height);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(0.0, static_cast<double>(g_client_width), static_cast<double>(g_client_height), 0.0, -1.0, 1.0);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
}

void draw_boid(const boids::Boid& boid, float size) {
    const float heading = std::atan2(boid.velocity.y, boid.velocity.x);
    const float side_angle = 2.45f;
    const float point_x = boid.position.x + std::cos(heading) * size;
    const float point_y = boid.position.y + std::sin(heading) * size;
    const float left_x = boid.position.x + std::cos(heading + side_angle) * size * 0.78f;
    const float left_y = boid.position.y + std::sin(heading + side_angle) * size * 0.78f;
    const float right_x = boid.position.x + std::cos(heading - side_angle) * size * 0.78f;
    const float right_y = boid.position.y + std::sin(heading - side_angle) * size * 0.78f;

    glVertex2f(point_x, point_y);
    glVertex2f(left_x, left_y);
    glVertex2f(right_x, right_y);
}

void render_frame() {
    if (g_simulation == nullptr || g_device_context == nullptr) {
        return;
    }

    const auto now = std::chrono::steady_clock::now();
    float elapsed = std::chrono::duration<float>(now - g_last_frame_time).count();
    g_last_frame_time = now;
    elapsed = std::clamp(elapsed, 0.0f, kMaximumFrameSeconds);
    g_accumulator += elapsed;

    while (g_accumulator >= kSimulationStepSeconds) {
        g_simulation->step(kSimulationStepSeconds);
        g_accumulator -= kSimulationStepSeconds;
    }

    update_projection();
    glClear(GL_COLOR_BUFFER_BIT);

    const auto& settings = g_simulation->settings();
    glBegin(GL_TRIANGLES);
    for (const boids::Boid& boid : g_simulation->boids()) {
        if (boid.is_predator) {
            glColor3f(0.96f, 0.20f, 0.16f);
            draw_boid(boid, settings.boid_size * 1.65f);
        } else {
            const float speed_ratio = std::min(1.0f, boid.velocity.length() / settings.max_speed);
            glColor3f(0.20f + 0.25f * speed_ratio, 0.66f + 0.28f * speed_ratio, 0.95f);
            draw_boid(boid, settings.boid_size);
        }
    }
    glEnd();

    SwapBuffers(g_device_context);
}

void request_exit() {
    if (g_window != nullptr) {
        DestroyWindow(g_window);
    }
}

void apply_adjustment_settings(void* context, const boids::FlockSettings& settings) {
    auto* const simulation = static_cast<boids::FlockSimulation*>(context);
    if (simulation == nullptr) {
        return;
    }

    simulation->apply_settings(settings);
    g_accumulator = 0.0f;
    g_last_frame_time = std::chrono::steady_clock::now();
}

void position_adjustment_panel(HWND panel) {
    RECT owner_rect{};
    RECT panel_rect{};
    if (!GetWindowRect(g_window, &owner_rect) || !GetWindowRect(panel, &panel_rect)) {
        return;
    }

    MONITORINFO monitor_info{};
    monitor_info.cbSize = sizeof(monitor_info);
    const HMONITOR monitor = MonitorFromWindow(g_window, MONITOR_DEFAULTTONEAREST);
    if (monitor == nullptr || !GetMonitorInfoA(monitor, &monitor_info)) {
        return;
    }

    const int panel_width = panel_rect.right - panel_rect.left;
    const int panel_height = panel_rect.bottom - panel_rect.top;
    const RECT work_area = monitor_info.rcWork;
    int x = owner_rect.right + 12;
    if (x + panel_width > work_area.right) {
        x = owner_rect.left - panel_width - 12;
    }
    const int minimum_x = static_cast<int>(work_area.left);
    const int maximum_x = std::max(minimum_x, static_cast<int>(work_area.right) - panel_width);
    x = std::clamp(x, minimum_x, maximum_x);
    const int minimum_y = static_cast<int>(work_area.top);
    const int maximum_y = std::max(minimum_y, static_cast<int>(work_area.bottom) - panel_height);
    const int y = std::clamp(static_cast<int>(owner_rect.top), minimum_y, maximum_y);
    SetWindowPos(panel, HWND_TOP, x, y, 0, 0, SWP_NOSIZE | SWP_NOACTIVATE);
}

LRESULT CALLBACK window_procedure(HWND window, UINT message, WPARAM w_param, LPARAM l_param) {
    switch (message) {
    case WM_CREATE:
        SetTimer(window, kFrameTimerId, kFrameTimerMilliseconds, nullptr);
        GetCursorPos(&g_initial_mouse);
        return 0;

    case WM_SIZE:
        get_client_size(window);
        if (g_simulation != nullptr) {
            g_simulation->resize(static_cast<float>(g_client_width), static_cast<float>(g_client_height));
        }
        return 0;

    case WM_TIMER:
        if (w_param == kFrameTimerId) {
            render_frame();
        }
        return 0;

    case WM_ERASEBKGND:
        return 1;

    case WM_KEYDOWN:
    case WM_SYSKEYDOWN:
    case WM_LBUTTONDOWN:
    case WM_RBUTTONDOWN:
    case WM_MBUTTONDOWN:
        if (g_exit_on_input) {
            request_exit();
        }
        return 0;

    case WM_MOUSEMOVE:
        if (g_exit_on_input) {
            const int x = GET_X_LPARAM(l_param);
            const int y = GET_Y_LPARAM(l_param);
            POINT current{ x, y };
            ClientToScreen(window, &current);
            if (std::abs(current.x - g_initial_mouse.x) > 4 || std::abs(current.y - g_initial_mouse.y) > 4) {
                request_exit();
            }
        }
        return 0;

    case WM_CLOSE:
        DestroyWindow(window);
        return 0;

    case WM_DESTROY:
        if (g_adjustment_panel != nullptr && IsWindow(g_adjustment_panel)) {
            DestroyWindow(g_adjustment_panel);
        }
        g_adjustment_panel = nullptr;
        KillTimer(window, kFrameTimerId);
        PostQuitMessage(0);
        return 0;

    default:
        return DefWindowProcA(window, message, w_param, l_param);
    }
}

bool register_window_class() {
    WNDCLASSEXA window_class{};
    window_class.cbSize = sizeof(window_class);
    window_class.style = CS_OWNDC | CS_HREDRAW | CS_VREDRAW;
    window_class.lpfnWndProc = window_procedure;
    window_class.hInstance = g_instance;
    window_class.hCursor = LoadCursor(nullptr, IDC_ARROW);
    window_class.hbrBackground = nullptr;
    window_class.lpszClassName = kWindowClass;
    return RegisterClassExA(&window_class) != 0 || GetLastError() == ERROR_CLASS_ALREADY_EXISTS;
}

HWND create_saver_window(HWND preview_parent) {
    DWORD style = WS_POPUP;
    DWORD extended_style = WS_EX_TOPMOST;
    int x = GetSystemMetrics(SM_XVIRTUALSCREEN);
    int y = GetSystemMetrics(SM_YVIRTUALSCREEN);
    int width = GetSystemMetrics(SM_CXVIRTUALSCREEN);
    int height = GetSystemMetrics(SM_CYVIRTUALSCREEN);
    HWND parent = nullptr;

    if (g_preview) {
        style = WS_CHILD | WS_VISIBLE;
        extended_style = 0;
        parent = preview_parent;
        RECT preview_rect{};
        GetClientRect(preview_parent, &preview_rect);
        width = std::max(1L, preview_rect.right - preview_rect.left);
        height = std::max(1L, preview_rect.bottom - preview_rect.top);
    } else if (g_adjustment_mode) {
        style = WS_OVERLAPPEDWINDOW | WS_VISIBLE;
        extended_style = 0;
        x = CW_USEDEFAULT;
        y = CW_USEDEFAULT;
        width = kAdjustmentWidth;
        height = kAdjustmentHeight;
    }

    return CreateWindowExA(
        extended_style,
        kWindowClass,
        g_adjustment_mode ? "Boids Screensaver — Live Adjustment" : kAppTitle,
        style,
        x,
        y,
        width,
        height,
        parent,
        nullptr,
        g_instance,
        nullptr);
}

bool command_has_no_arguments(const char* text) {
    while (*text == ' ' || *text == '\t' || *text == ':') {
        ++text;
    }
    return *text == '\0';
}

bool parse_configuration_owner(char* text, HWND* owner) {
    while (*text == ' ' || *text == '\t' || *text == ':') {
        ++text;
    }
    if (*text == '\0') {
        *owner = nullptr;
        return true;
    }

    char* end = nullptr;
    const unsigned long long raw = std::strtoull(text, &end, 0);
    while (*end == ' ' || *end == '\t') {
        ++end;
    }
    if (end == text || *end != '\0') {
        return false;
    }

    const HWND parsed_owner = reinterpret_cast<HWND>(static_cast<uintptr_t>(raw));
    *owner = IsWindow(parsed_owner) != FALSE ? parsed_owner : nullptr;
    return true;
}

}  // namespace

int WINAPI WinMain(HINSTANCE instance, HINSTANCE, LPSTR command_line, int) {
    g_instance = instance;

    char command[256]{};
    std::strncpy(command, command_line == nullptr ? "" : command_line, sizeof(command) - 1);
    char* cursor = command;
    while (*cursor == ' ' || *cursor == '\t') {
        ++cursor;
    }

    HWND preview_parent = nullptr;
    HWND configuration_owner = nullptr;
    if (*cursor == '\0') {
        boids::show_control_panel(g_instance);
        return 0;
    }
    if (_strnicmp(cursor, "/c", 2) == 0) {
        if (!parse_configuration_owner(cursor + 2, &configuration_owner)) {
            message_box("Configuration mode accepts only an optional window handle.");
            return 1;
        }
        boids::show_control_panel(g_instance, configuration_owner);
        return 0;
    }
    if (_strnicmp(cursor, "/a", 2) == 0) {
        return 0;
    }
    if (_strnicmp(cursor, "/p", 2) == 0) {
        if (!parse_preview_parent(cursor + 2, &preview_parent)) {
            message_box("Preview mode requires a valid parent window handle.");
            return 1;
        }
        g_preview = true;
    } else if (_strnicmp(cursor, "/w", 2) == 0 && command_has_no_arguments(cursor + 2)) {
        g_adjustment_mode = true;
    } else if (_strnicmp(cursor, "/s", 2) == 0 || _strnicmp(cursor, "/d", 2) == 0) {
        if (!command_has_no_arguments(cursor + 2)) {
            message_box("Unsupported command-line arguments.");
            return 1;
        }
        g_fullscreen = true;
    } else {
        message_box("Supported modes: /s, /p <HWND>, /c, /w, /d.");
        return 1;
    }

    g_exit_on_input = g_fullscreen;
    if (!register_window_class()) {
        message_box("Could not register the screen-saver window class.");
        return 1;
    }

    g_window = create_saver_window(preview_parent);
    if (g_window == nullptr) {
        message_box("Could not create the screen-saver window.");
        return 1;
    }

    get_client_size(g_window);
    boids::FlockSettings settings = boids::load_control_panel_settings();
    if (g_preview) {
        // Keep Windows' tiny control-panel preview inexpensive regardless of
        // the user's full-screen population setting.
        settings.initial_flock_size = 120;
        settings.boid_size = 2.5f;
    }
    g_simulation = new boids::FlockSimulation(
        static_cast<float>(g_client_width), static_cast<float>(g_client_height), settings,
        static_cast<std::uint32_t>(GetTickCount()));

    if (!initialize_opengl(g_window)) {
        delete g_simulation;
        g_simulation = nullptr;
        DestroyWindow(g_window);
        message_box("Could not create an OpenGL rendering context.");
        return 1;
    }

    g_last_frame_time = std::chrono::steady_clock::now();
    ShowWindow(g_window, g_fullscreen ? SW_SHOWNOACTIVATE : SW_SHOW);
    UpdateWindow(g_window);
    if (g_adjustment_mode) {
        g_adjustment_panel = boids::create_adjustment_panel(
            g_instance, g_window, g_simulation->settings(), apply_adjustment_settings, g_simulation);
        if (g_adjustment_panel != nullptr) {
            position_adjustment_panel(g_adjustment_panel);
            ShowWindow(g_adjustment_panel, SW_SHOW);
        }
    }

    MSG message{};
    while (GetMessageA(&message, nullptr, 0, 0) > 0) {
        if (g_adjustment_panel != nullptr && !IsWindow(g_adjustment_panel)) {
            g_adjustment_panel = nullptr;
        }
        if (g_adjustment_panel != nullptr && IsDialogMessageA(g_adjustment_panel, &message)) {
            continue;
        }
        TranslateMessage(&message);
        DispatchMessageA(&message);
    }

    shutdown_opengl();
    delete g_simulation;
    g_simulation = nullptr;
    g_window = nullptr;
    return 0;
}
