#pragma once

#define WIN32_LEAN_AND_MEAN
#include <windows.h>

#include "flock_simulation.h"

namespace boids {

// Reads saved control-panel settings from HKCU. Missing or malformed values
// preserve compiled defaults.
[[nodiscard]] FlockSettings load_control_panel_settings();

// Displays the modal /c control panel. The optional owner is supplied by some
// Windows screen-saver control-panel invocations. Accepted values are validated
// and saved for later screen-saver, debug-window, and preview launches.
void show_control_panel(HINSTANCE instance, HWND owner = nullptr);

}  // namespace boids
