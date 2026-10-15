#include "settings_dialog.h"

#include <array>
#include <cerrno>
#include <climits>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <new>

#include "settings_config.h"
#include "settings_resource.h"

namespace boids {
namespace {

constexpr char kRegistryPath[] = "Software\\Qink\\BoidsScreensaver";
constexpr char kSolidWallsValue[] = "SolidWalls";
constexpr char kFlockSizeValue[] = "FlockSize";
constexpr char kPredatorCountValue[] = "PredatorCount";
constexpr char kMaxSpeedValue[] = "MaxSpeed";
constexpr char kPredatorSpeedMultiplierValue[] = "PredatorSpeedMultiplier";
constexpr char kWallTurnDistanceValue[] = "WallTurnDistance";
constexpr char kWallForceMultiplierValue[] = "WallForceMultiplier";
constexpr std::size_t kControlTextLength = 64;

struct SettingsDialogState {
    FlockSettings settings;
    SettingsSavedCallback on_saved = nullptr;
    void* context = nullptr;
    bool modeless = false;
};

bool read_dword(HKEY key, const char* name, DWORD* value) {
    DWORD type = 0;
    DWORD byte_count = sizeof(*value);
    return RegQueryValueExA(key, name, nullptr, &type,
                            reinterpret_cast<BYTE*>(value), &byte_count) == ERROR_SUCCESS &&
        type == REG_DWORD && byte_count == sizeof(*value);
}

bool read_float(HKEY key, const char* name, float* value) {
    std::array<char, kControlTextLength> text{};
    DWORD type = 0;
    DWORD byte_count = static_cast<DWORD>(text.size());
    if (RegQueryValueExA(key, name, nullptr, &type, reinterpret_cast<BYTE*>(text.data()),
                         &byte_count) != ERROR_SUCCESS || type != REG_SZ || byte_count == 0 ||
        byte_count > text.size()) {
        return false;
    }

    if (text[byte_count - 1] != '\0') {
        return false;
    }
    char* end = nullptr;
    errno = 0;
    const float parsed = std::strtof(text.data(), &end);
    if (end == text.data() || errno == ERANGE || !std::isfinite(parsed)) {
        return false;
    }
    while (*end == ' ' || *end == '\t') {
        ++end;
    }
    if (*end != '\0') {
        return false;
    }

    *value = parsed;
    return true;
}

bool write_dword(HKEY key, const char* name, DWORD value) {
    return RegSetValueExA(key, name, 0, REG_DWORD,
                          reinterpret_cast<const BYTE*>(&value), sizeof(value)) == ERROR_SUCCESS;
}

bool write_float(HKEY key, const char* name, float value) {
    std::array<char, kControlTextLength> text{};
    const int length = std::snprintf(text.data(), text.size(), "%.3f", value);
    if (length < 0 || static_cast<std::size_t>(length) >= text.size()) {
        return false;
    }
    return RegSetValueExA(key, name, 0, REG_SZ,
                          reinterpret_cast<const BYTE*>(text.data()),
                          static_cast<DWORD>(length + 1)) == ERROR_SUCCESS;
}

void set_control_text(HWND dialog, int control_id, const char* text) {
    SetDlgItemTextA(dialog, control_id, text);
}

void set_control_integer(HWND dialog, int control_id, int value) {
    char text[kControlTextLength]{};
    std::snprintf(text, sizeof(text), "%d", value);
    set_control_text(dialog, control_id, text);
}

void set_control_float(HWND dialog, int control_id, float value) {
    char text[kControlTextLength]{};
    std::snprintf(text, sizeof(text), "%.3f", value);
    set_control_text(dialog, control_id, text);
}

void populate_controls(HWND dialog, const FlockSettings& settings) {
    CheckDlgButton(dialog, IDC_ENABLE_SOLID_WALLS,
                   settings.wrap_edges ? BST_UNCHECKED : BST_CHECKED);
    set_control_integer(dialog, IDC_FLOCK_SIZE, settings.initial_flock_size);
    set_control_integer(dialog, IDC_PREDATOR_COUNT, settings.initial_predator_count);
    set_control_float(dialog, IDC_MAX_SPEED, settings.max_speed);
    set_control_float(dialog, IDC_PREDATOR_SPEED_MULTIPLIER, settings.predator_speed_multiplier);
    set_control_float(dialog, IDC_WALL_TURN_DISTANCE, settings.wall_turn_distance);
    set_control_float(dialog, IDC_WALL_FORCE_MULTIPLIER, settings.wall_force_multiplier);
}

bool read_control_text(HWND dialog, int control_id, std::array<char, kControlTextLength>* text) {
    const HWND control = GetDlgItem(dialog, control_id);
    if (control == nullptr) {
        return false;
    }
    const int length = GetWindowTextA(control, text->data(), static_cast<int>(text->size()));
    return length > 0 && static_cast<std::size_t>(length) < text->size();
}

bool parse_integer_text(const char* text, int* value) {
    char* end = nullptr;
    errno = 0;
    const long parsed = std::strtol(text, &end, 10);
    if (end == text || errno == ERANGE) {
        return false;
    }
    while (*end == ' ' || *end == '\t') {
        ++end;
    }
    if (*end != '\0' || parsed < static_cast<long>(INT_MIN) ||
        parsed > static_cast<long>(INT_MAX)) {
        return false;
    }

    *value = static_cast<int>(parsed);
    return true;
}

bool parse_float_text(const char* text, float* value) {
    char* end = nullptr;
    errno = 0;
    const float parsed = std::strtof(text, &end);
    if (end == text || errno == ERANGE || !std::isfinite(parsed)) {
        return false;
    }
    while (*end == ' ' || *end == '\t') {
        ++end;
    }
    if (*end != '\0') {
        return false;
    }

    *value = parsed;
    return true;
}

bool is_in_range(int value, int minimum, int maximum) {
    return value >= minimum && value <= maximum;
}

bool is_in_range(float value, float minimum, float maximum) {
    return value >= minimum && value <= maximum;
}

bool read_controls(HWND dialog, FlockSettings* settings) {
    FlockSettings candidate = *settings;
    candidate.wrap_edges = IsDlgButtonChecked(dialog, IDC_ENABLE_SOLID_WALLS) != BST_CHECKED;

    std::array<char, kControlTextLength> flock_size_text{};
    std::array<char, kControlTextLength> predator_count_text{};
    std::array<char, kControlTextLength> max_speed_text{};
    std::array<char, kControlTextLength> predator_speed_text{};
    std::array<char, kControlTextLength> wall_distance_text{};
    std::array<char, kControlTextLength> wall_force_text{};
    const bool flock_size_read = read_control_text(dialog, IDC_FLOCK_SIZE, &flock_size_text);
    const bool predator_count_read = read_control_text(
        dialog, IDC_PREDATOR_COUNT, &predator_count_text);
    const bool max_speed_read = read_control_text(dialog, IDC_MAX_SPEED, &max_speed_text);
    const bool predator_speed_read = read_control_text(
        dialog, IDC_PREDATOR_SPEED_MULTIPLIER, &predator_speed_text);
    const bool wall_distance_read = read_control_text(
        dialog, IDC_WALL_TURN_DISTANCE, &wall_distance_text);
    const bool wall_force_read = read_control_text(
        dialog, IDC_WALL_FORCE_MULTIPLIER, &wall_force_text);
    const bool flock_size_valid = flock_size_read &&
        parse_integer_text(flock_size_text.data(), &candidate.initial_flock_size);
    const bool predator_count_valid = predator_count_read &&
        parse_integer_text(predator_count_text.data(), &candidate.initial_predator_count);
    const bool max_speed_valid = max_speed_read &&
        parse_float_text(max_speed_text.data(), &candidate.max_speed);
    const bool predator_speed_valid = predator_speed_read &&
        parse_float_text(predator_speed_text.data(), &candidate.predator_speed_multiplier);
    const bool wall_distance_valid = wall_distance_read &&
        parse_float_text(wall_distance_text.data(), &candidate.wall_turn_distance);
    const bool wall_force_valid = wall_force_read &&
        parse_float_text(wall_force_text.data(), &candidate.wall_force_multiplier);

    if (!flock_size_valid || !predator_count_valid || !max_speed_valid || !predator_speed_valid ||
        !wall_distance_valid || !wall_force_valid) {
        MessageBoxA(dialog, "Enter a number in every field.", "Boids Screensaver", MB_OK | MB_ICONWARNING);
        return false;
    }

    if (!is_in_range(candidate.initial_flock_size, kMinimumFlockSize, kMaximumFlockSize) ||
        !is_in_range(candidate.initial_predator_count, kMinimumPredatorCount, kMaximumPredatorCount) ||
        !is_in_range(candidate.max_speed, kMinimumBoidSpeed, kMaximumBoidSpeed) ||
        !is_in_range(candidate.predator_speed_multiplier, kMinimumPredatorSpeedMultiplier,
                     kMaximumPredatorSpeedMultiplier) ||
        !is_in_range(candidate.wall_turn_distance, kMinimumWallTurnDistance, kMaximumWallTurnDistance) ||
        !is_in_range(candidate.wall_force_multiplier, kMinimumWallForceMultiplier,
                     kMaximumWallForceMultiplier)) {
        MessageBoxA(dialog,
                    "Use values within the ranges displayed beside each field.",
                    "Boids Screensaver", MB_OK | MB_ICONWARNING);
        return false;
    }

    *settings = candidate;
    return true;
}

bool save_control_panel_settings(const FlockSettings& settings) {
    HKEY key = nullptr;
    if (RegCreateKeyExA(HKEY_CURRENT_USER, kRegistryPath, 0, nullptr, 0,
                        KEY_SET_VALUE, nullptr, &key, nullptr) != ERROR_SUCCESS) {
        return false;
    }

    const bool saved =
        write_dword(key, kSolidWallsValue, settings.wrap_edges ? 0u : 1u) &&
        write_dword(key, kFlockSizeValue, static_cast<DWORD>(settings.initial_flock_size)) &&
        write_dword(key, kPredatorCountValue, static_cast<DWORD>(settings.initial_predator_count)) &&
        write_float(key, kMaxSpeedValue, settings.max_speed) &&
        write_float(key, kPredatorSpeedMultiplierValue, settings.predator_speed_multiplier) &&
        write_float(key, kWallTurnDistanceValue, settings.wall_turn_distance) &&
        write_float(key, kWallForceMultiplierValue, settings.wall_force_multiplier);
    RegCloseKey(key);
    return saved;
}

SettingsDialogState* dialog_state(HWND dialog) {
    return reinterpret_cast<SettingsDialogState*>(
        GetWindowLongPtrA(dialog, GWLP_USERDATA));
}

void close_settings_dialog(HWND dialog, SettingsDialogState* state, INT_PTR result) {
    if (state->modeless) {
        DestroyWindow(dialog);
    } else {
        EndDialog(dialog, result);
    }
}

INT_PTR CALLBACK settings_dialog_procedure(HWND dialog, UINT message, WPARAM w_param, LPARAM l_param) {
    switch (message) {
    case WM_INITDIALOG: {
        auto* const state = reinterpret_cast<SettingsDialogState*>(l_param);
        if (state == nullptr) {
            return FALSE;
        }
        SetWindowLongPtrA(dialog, GWLP_USERDATA, reinterpret_cast<LONG_PTR>(state));
        populate_controls(dialog, state->settings);
        return TRUE;
    }

    case WM_COMMAND: {
        SettingsDialogState* const state = dialog_state(dialog);
        if (state == nullptr) {
            return FALSE;
        }
        switch (LOWORD(w_param)) {
        case IDC_RESTORE_DEFAULTS:
            populate_controls(dialog, FlockSettings{});
            return TRUE;

        case IDOK: {
            FlockSettings settings = state->settings;
            if (!read_controls(dialog, &settings)) {
                return TRUE;
            }
            if (!save_control_panel_settings(settings)) {
                MessageBoxA(dialog, "Could not save settings to the current-user registry.",
                            "Boids Screensaver", MB_OK | MB_ICONERROR);
                return TRUE;
            }
            state->settings = settings;
            if (state->on_saved != nullptr) {
                state->on_saved(state->context, settings);
            }
            if (!state->modeless) {
                close_settings_dialog(dialog, state, IDOK);
            }
            return TRUE;
        }

        case IDCANCEL:
            close_settings_dialog(dialog, state, IDCANCEL);
            return TRUE;

        default:
            return FALSE;
        }
    }

    case WM_CLOSE: {
        SettingsDialogState* const state = dialog_state(dialog);
        if (state != nullptr) {
            close_settings_dialog(dialog, state, IDCANCEL);
            return TRUE;
        }
        return FALSE;
    }

    case WM_NCDESTROY: {
        SettingsDialogState* const state = dialog_state(dialog);
        SetWindowLongPtrA(dialog, GWLP_USERDATA, 0);
        if (state != nullptr && state->modeless) {
            delete state;
        }
        return FALSE;
    }

    default:
        return FALSE;
    }
}

}  // namespace

FlockSettings load_control_panel_settings() {
    FlockSettings settings;
    HKEY key = nullptr;
    if (RegOpenKeyExA(HKEY_CURRENT_USER, kRegistryPath, 0, KEY_QUERY_VALUE, &key) != ERROR_SUCCESS) {
        return settings;
    }

    DWORD value = 0;
    if (read_dword(key, kSolidWallsValue, &value) && (value == 0 || value == 1)) {
        settings.wrap_edges = value == 0;
    }
    if (read_dword(key, kFlockSizeValue, &value) && value <= static_cast<DWORD>(INT_MAX)) {
        settings.initial_flock_size = static_cast<int>(value);
    }
    if (read_dword(key, kPredatorCountValue, &value) && value <= static_cast<DWORD>(INT_MAX)) {
        settings.initial_predator_count = static_cast<int>(value);
    }
    read_float(key, kMaxSpeedValue, &settings.max_speed);
    read_float(key, kPredatorSpeedMultiplierValue, &settings.predator_speed_multiplier);
    read_float(key, kWallTurnDistanceValue, &settings.wall_turn_distance);
    read_float(key, kWallForceMultiplierValue, &settings.wall_force_multiplier);
    RegCloseKey(key);
    return sanitize_control_panel_settings(settings);
}

void show_control_panel(HINSTANCE instance, HWND owner) {
    SettingsDialogState state{};
    state.settings = load_control_panel_settings();
    const INT_PTR result = DialogBoxParamA(instance, MAKEINTRESOURCEA(IDD_SETTINGS_DIALOG), owner,
                                           settings_dialog_procedure,
                                           reinterpret_cast<LPARAM>(&state));
    if (result == -1) {
        char message[128]{};
        std::snprintf(message, sizeof(message), "Could not open settings dialog (error %lu).",
                      GetLastError());
        MessageBoxA(owner, message, "Boids Screensaver", MB_OK | MB_ICONERROR);
    }
}

HWND create_adjustment_panel(HINSTANCE instance, HWND owner,
                             const FlockSettings& initial_settings,
                             SettingsSavedCallback on_saved, void* context) {
    auto* const state = new (std::nothrow) SettingsDialogState{
        initial_settings, on_saved, context, true};
    if (state == nullptr) {
        MessageBoxA(owner, "Could not allocate the live settings panel.",
                    "Boids Screensaver", MB_OK | MB_ICONERROR);
        return nullptr;
    }

    const HWND panel = CreateDialogParamA(instance, MAKEINTRESOURCEA(IDD_SETTINGS_DIALOG), owner,
                                          settings_dialog_procedure,
                                          reinterpret_cast<LPARAM>(state));
    if (panel == nullptr) {
        const DWORD error = GetLastError();
        delete state;
        char message[128]{};
        std::snprintf(message, sizeof(message), "Could not open settings panel (error %lu).", error);
        MessageBoxA(owner, message, "Boids Screensaver", MB_OK | MB_ICONERROR);
        return nullptr;
    }

    return panel;
}

}  // namespace boids
