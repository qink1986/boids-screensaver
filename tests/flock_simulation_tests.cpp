#include <cmath>
#include <cstdint>
#include <iostream>
#include <stdexcept>

#include "flock_simulation.h"

namespace {

void require(bool condition, const char* message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

bool approximately_equal(float left, float right, float tolerance = 1.0e-4f) {
    return std::abs(left - right) <= tolerance;
}

void require_valid_flock(const boids::FlockSimulation& simulation) {
    const auto& settings = simulation.settings();
    for (const boids::Boid& boid : simulation.boids()) {
        require(std::isfinite(boid.position.x), "boid x position must be finite");
        require(std::isfinite(boid.position.y), "boid y position must be finite");
        require(std::isfinite(boid.velocity.x), "boid x velocity must be finite");
        require(std::isfinite(boid.velocity.y), "boid y velocity must be finite");
        require(boid.position.x >= 0.0f && boid.position.x < simulation.width(),
                "boid x position must remain in the world");
        require(boid.position.y >= 0.0f && boid.position.y < simulation.height(),
                "boid y position must remain in the world");
        require(boid.velocity.length() <= settings.max_speed + 1.0e-3f,
                "boid speed must not exceed max_speed");
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

void test_long_running_wrapped_flock() {
    boids::FlockSettings settings;
    settings.initial_flock_size = 1024;
    boids::FlockSimulation simulation(1280.0f, 720.0f, settings, 0xCAFEBABEu);

    for (int step = 0; step < 600; ++step) {
        simulation.step(1.0f / 60.0f);
    }

    require_valid_flock(simulation);
}

void test_resize_and_empty_flock() {
    boids::FlockSettings settings;
    settings.initial_flock_size = 0;
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
        test_long_running_wrapped_flock();
        test_resize_and_empty_flock();
        std::cout << "flock simulation tests: PASS\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "flock simulation tests: FAIL: " << error.what() << '\n';
        return 1;
    }
}
