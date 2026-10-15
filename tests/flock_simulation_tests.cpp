#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <stdexcept>

#include "flock_simulation.h"
#include "settings_config.h"

namespace {

void require(bool condition, const char* message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

bool approximately_equal(float left, float right, float tolerance = 1.0e-4f) {
    return std::abs(left - right) <= tolerance;
}

int predator_count(const boids::FlockSimulation& simulation) {
    int count = 0;
    for (const boids::Boid& boid : simulation.boids()) {
        count += boid.is_predator ? 1 : 0;
    }
    return count;
}

void require_valid_flock(const boids::FlockSimulation& simulation) {
    const auto& settings = simulation.settings();
    for (const boids::Boid& boid : simulation.boids()) {
        const float speed_limit = settings.max_speed *
            (boid.is_predator ? settings.predator_speed_multiplier : 1.0f);
        require(std::isfinite(boid.position.x), "boid x position must be finite");
        require(std::isfinite(boid.position.y), "boid y position must be finite");
        require(std::isfinite(boid.velocity.x), "boid x velocity must be finite");
        require(std::isfinite(boid.velocity.y), "boid y velocity must be finite");
        require(boid.position.x >= 0.0f && boid.position.x < simulation.width(),
                "boid x position must remain in the world");
        require(boid.position.y >= 0.0f && boid.position.y < simulation.height(),
                "boid y position must remain in the world");
        require(boid.velocity.length() <= speed_limit + 1.0e-3f,
                "boid speed must not exceed its speed limit");
    }
}

void test_deterministic_fixed_step() {
    boids::FlockSettings settings;
    settings.initial_flock_size = 256;
    boids::FlockSimulation first(800.0f, 600.0f, settings, 0x12345678u);
    boids::FlockSimulation second(800.0f, 600.0f, settings, 0x12345678u);

    for (int step = 0; step < 240; ++step) {
        first.step(1.0f / 60.0f);
        second.step(1.0f / 60.0f);
    }

    const auto& first_boids = first.boids();
    const auto& second_boids = second.boids();
    require(first_boids.size() == second_boids.size(), "same seed must create same flock size");
    require(predator_count(first) == settings.initial_predator_count,
            "default flock must include three predators");
    for (std::size_t index = 0; index < first_boids.size(); ++index) {
        require(approximately_equal(first_boids[index].position.x, second_boids[index].position.x),
                "fixed-step x evolution must be deterministic");
        require(approximately_equal(first_boids[index].position.y, second_boids[index].position.y),
                "fixed-step y evolution must be deterministic");
        require(approximately_equal(first_boids[index].velocity.x, second_boids[index].velocity.x),
                "fixed-step x velocity must be deterministic");
        require(approximately_equal(first_boids[index].velocity.y, second_boids[index].velocity.y),
                "fixed-step y velocity must be deterministic");
    }

    require_valid_flock(first);
}

void test_predator_population() {
    boids::FlockSettings settings;
    settings.initial_flock_size = 64;
    settings.initial_predator_count = 3;
    boids::FlockSimulation simulation(800.0f, 600.0f, settings, 0xF00DFACEu);

    require(simulation.boids().size() == 67, "flock size must include three predators");
    require(predator_count(simulation) == 3, "simulation must spawn exactly three predators");
    for (const boids::Boid& boid : simulation.boids()) {
        const float expected_speed = settings.max_speed *
            (boid.is_predator ? settings.predator_speed_multiplier : 1.0f);
        require(approximately_equal(boid.velocity.length(), expected_speed, 1.0e-3f),
                "predator and prey must spawn at their configured speed limits");
    }

    for (int step = 0; step < 360; ++step) {
        simulation.step(1.0f / 60.0f);
    }
    require(predator_count(simulation) == 3, "predator population must remain stable");
    require_valid_flock(simulation);
}

void test_long_running_wrapped_flock() {
    boids::FlockSettings settings;
    settings.initial_flock_size = 1024;
    settings.wrap_edges = true;
    boids::FlockSimulation simulation(1280.0f, 720.0f, settings, 0xCAFEBABEu);

    for (int step = 0; step < 600; ++step) {
        simulation.step(1.0f / 60.0f);
    }

    require_valid_flock(simulation);
}

void require_inside_physical_walls(const boids::FlockSimulation& simulation) {
    for (const boids::Boid& boid : simulation.boids()) {
        require(boid.position.x >= 0.0f, "solid wall must keep boid inside left edge");
        require(boid.position.x < simulation.width(), "solid wall must keep boid inside right edge");
        require(boid.position.y >= 0.0f, "solid wall must keep boid inside top edge");
        require(boid.position.y < simulation.height(), "solid wall must keep boid inside bottom edge");
    }
}

void test_solid_wall_flock() {
    boids::FlockSettings settings;
    settings.initial_flock_size = 256;
    settings.initial_predator_count = 3;
    require(!settings.wrap_edges, "solid walls must be the default boundary mode");
    boids::FlockSimulation simulation(1280.0f, 720.0f, settings, 0xB011DAB1u);

    require_inside_physical_walls(simulation);
    for (int step = 0; step < 600; ++step) {
        simulation.step(1.0f / 60.0f);
        require_inside_physical_walls(simulation);
    }

    require_valid_flock(simulation);
}

void test_dense_capped_flock() {
    boids::FlockSettings settings;
    settings.initial_flock_size = 128;
    settings.initial_predator_count = 3;
    settings.max_sampled_prey_candidates = 16;
    settings.wrap_edges = true;
    boids::FlockSimulation first(48.0f, 48.0f, settings, 0xD3E5E123u);
    boids::FlockSimulation second(48.0f, 48.0f, settings, 0xD3E5E123u);

    for (int step = 0; step < 180; ++step) {
        first.step(1.0f / 60.0f);
        second.step(1.0f / 60.0f);
    }

    require_valid_flock(first);
    require(predator_count(first) == settings.initial_predator_count,
            "dense flock must preserve predators");
    const auto& first_boids = first.boids();
    const auto& second_boids = second.boids();
    for (std::size_t index = 0; index < first_boids.size(); ++index) {
        require(approximately_equal(first_boids[index].position.x, second_boids[index].position.x),
                "capped dense flock x evolution must be deterministic");
        require(approximately_equal(first_boids[index].position.y, second_boids[index].position.y),
                "capped dense flock y evolution must be deterministic");
        require(approximately_equal(first_boids[index].velocity.x, second_boids[index].velocity.x),
                "capped dense flock x velocity must be deterministic");
        require(approximately_equal(first_boids[index].velocity.y, second_boids[index].velocity.y),
                "capped dense flock y velocity must be deterministic");
    }
}

void test_control_panel_setting_limits() {
    boids::FlockSettings settings;
    settings.initial_flock_size = -1;
    settings.initial_predator_count = 999;
    settings.max_speed = std::numeric_limits<float>::quiet_NaN();
    settings.predator_speed_multiplier = 99.0f;
    settings.wall_turn_distance = -5.0f;
    settings.wall_force_multiplier = -1.0f;

    const boids::FlockSettings sanitized = boids::sanitize_control_panel_settings(settings);
    require(sanitized.initial_flock_size == boids::kMinimumFlockSize,
            "control panel must clamp flock count");
    require(sanitized.initial_predator_count == boids::kMaximumPredatorCount,
            "control panel must clamp predator count");
    require(approximately_equal(sanitized.max_speed, boids::FlockSettings{}.max_speed),
            "control panel must restore a finite default bird speed");
    require(approximately_equal(sanitized.predator_speed_multiplier,
                                boids::kMaximumPredatorSpeedMultiplier),
            "control panel must clamp predator speed ratio");
    require(approximately_equal(sanitized.wall_turn_distance, boids::kMinimumWallTurnDistance),
            "control panel must clamp soft wall range");
    require(approximately_equal(sanitized.wall_force_multiplier,
                                boids::kMinimumWallForceMultiplier),
            "control panel must clamp soft wall strength");

    settings.initial_flock_size = 2400;
    settings.initial_predator_count = 7;
    settings.max_speed = 525.0f;
    settings.predator_speed_multiplier = 1.75f;
    settings.wall_turn_distance = 180.0f;
    settings.wall_force_multiplier = 0.6f;
    const boids::FlockSettings preserved = boids::sanitize_control_panel_settings(settings);
    require(preserved.initial_flock_size == settings.initial_flock_size,
            "control panel must preserve valid flock count");
    require(preserved.initial_predator_count == settings.initial_predator_count,
            "control panel must preserve valid predator count");
    require(approximately_equal(preserved.max_speed, settings.max_speed),
            "control panel must preserve valid bird speed");
    require(approximately_equal(preserved.predator_speed_multiplier,
                                settings.predator_speed_multiplier),
            "control panel must preserve valid predator speed ratio");
    require(approximately_equal(preserved.wall_turn_distance, settings.wall_turn_distance),
            "control panel must preserve valid soft wall range");
    require(approximately_equal(preserved.wall_force_multiplier, settings.wall_force_multiplier),
            "control panel must preserve valid soft wall strength");
}

void test_apply_settings_recreates_flock() {
    boids::FlockSettings original;
    original.initial_flock_size = 8;
    original.initial_predator_count = 1;
    boids::FlockSimulation simulation(800.0f, 600.0f, original, 0xACCE551u);

    boids::FlockSettings adjusted = original;
    adjusted.initial_flock_size = 23;
    adjusted.initial_predator_count = 4;
    adjusted.max_speed = 525.0f;
    adjusted.predator_speed_multiplier = 1.75f;
    adjusted.wall_turn_distance = 180.0f;
    adjusted.wall_force_multiplier = 0.6f;
    adjusted.wrap_edges = true;
    simulation.apply_settings(adjusted);

    const boids::FlockSettings& active = simulation.settings();
    require(active.initial_flock_size == adjusted.initial_flock_size,
            "applying settings must update flock size");
    require(active.initial_predator_count == adjusted.initial_predator_count,
            "applying settings must update predator count");
    require(approximately_equal(active.max_speed, adjusted.max_speed),
            "applying settings must update bird speed");
    require(approximately_equal(active.predator_speed_multiplier,
                                adjusted.predator_speed_multiplier),
            "applying settings must update predator speed ratio");
    require(active.wrap_edges == adjusted.wrap_edges,
            "applying settings must update boundary mode");
    require(simulation.boids().size() == 27,
            "applying settings must recreate the configured prey and predators");
    require(predator_count(simulation) == 4,
            "applying settings must recreate the configured predator population");

    simulation.step(1.0f / 60.0f);
    require_valid_flock(simulation);
}

void test_resize_and_empty_flock() {
    boids::FlockSettings settings;
    settings.initial_flock_size = 0;
    settings.initial_predator_count = 0;
    boids::FlockSimulation simulation(400.0f, 300.0f, settings, 17u);
    simulation.step(1.0f / 60.0f);
    require(simulation.boids().empty(), "empty flock must remain empty");

    simulation.reset(32);
    simulation.resize(96.0f, 64.0f);
    for (int step = 0; step < 120; ++step) {
        simulation.step(1.0f / 60.0f);
    }
    require_valid_flock(simulation);
}

}  // namespace

int main() {
    try {
        test_deterministic_fixed_step();
        test_predator_population();
        test_long_running_wrapped_flock();
        test_solid_wall_flock();
        test_dense_capped_flock();
        test_control_panel_setting_limits();
        test_apply_settings_recreates_flock();
        test_resize_and_empty_flock();
        std::cout << "flock simulation tests: PASS\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "flock simulation tests: FAIL: " << error.what() << '\n';
        return 1;
    }
}
