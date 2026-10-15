#include "settings_config.h"

#include <algorithm>
#include <cmath>

namespace boids {
namespace {

template <typename T>
T sanitize_finite(T value, T fallback, T minimum, T maximum) {
    if (!std::isfinite(value)) {
        return fallback;
    }
    return std::clamp(value, minimum, maximum);
}

}  // namespace

FlockSettings sanitize_control_panel_settings(FlockSettings settings) {
    const FlockSettings defaults;
    settings.initial_flock_size = std::clamp(
        settings.initial_flock_size, kMinimumFlockSize, kMaximumFlockSize);
    settings.initial_predator_count = std::clamp(
        settings.initial_predator_count, kMinimumPredatorCount, kMaximumPredatorCount);
    settings.max_speed = sanitize_finite(
        settings.max_speed, defaults.max_speed, kMinimumBoidSpeed, kMaximumBoidSpeed);
    settings.predator_speed_multiplier = sanitize_finite(
        settings.predator_speed_multiplier, defaults.predator_speed_multiplier,
        kMinimumPredatorSpeedMultiplier, kMaximumPredatorSpeedMultiplier);
    settings.wall_turn_distance = sanitize_finite(
        settings.wall_turn_distance, defaults.wall_turn_distance,
        kMinimumWallTurnDistance, kMaximumWallTurnDistance);
    settings.wall_force_multiplier = sanitize_finite(
        settings.wall_force_multiplier, defaults.wall_force_multiplier,
        kMinimumWallForceMultiplier, kMaximumWallForceMultiplier);
    return settings;
}

}  // namespace boids
