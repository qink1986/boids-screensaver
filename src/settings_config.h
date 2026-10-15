#pragma once

#include "flock_simulation.h"

namespace boids {

constexpr int kMinimumFlockSize = 0;
constexpr int kMaximumFlockSize = 5000;
constexpr int kMinimumPredatorCount = 0;
constexpr int kMaximumPredatorCount = 20;
constexpr float kMinimumBoidSpeed = 50.0f;
constexpr float kMaximumBoidSpeed = 800.0f;
constexpr float kMinimumPredatorSpeedMultiplier = 1.0f;
constexpr float kMaximumPredatorSpeedMultiplier = 3.0f;
constexpr float kMinimumWallTurnDistance = 0.0f;
constexpr float kMaximumWallTurnDistance = 500.0f;
constexpr float kMinimumWallForceMultiplier = 0.0f;
constexpr float kMaximumWallForceMultiplier = 1.0f;

// Applies limits exposed by the Windows control panel. Other simulation
// settings retain their existing values.
[[nodiscard]] FlockSettings sanitize_control_panel_settings(FlockSettings settings);

}  // namespace boids
