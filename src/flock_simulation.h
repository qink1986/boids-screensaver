#pragma once

#include <cstdint>
#include <vector>

namespace boids {

struct Vec2 {
    float x = 0.0f;
    float y = 0.0f;

    Vec2() = default;
    Vec2(float x_value, float y_value) : x(x_value), y(y_value) {}

    Vec2& operator+=(const Vec2& other);
    Vec2& operator-=(const Vec2& other);
    Vec2& operator*=(float scalar);

    [[nodiscard]] float length_squared() const;
    [[nodiscard]] float length() const;
    [[nodiscard]] Vec2 normalized() const;
    [[nodiscard]] Vec2 limited(float maximum) const;
};

[[nodiscard]] Vec2 operator+(Vec2 left, const Vec2& right);
[[nodiscard]] Vec2 operator-(Vec2 left, const Vec2& right);
[[nodiscard]] Vec2 operator*(Vec2 value, float scalar);
[[nodiscard]] Vec2 operator*(float scalar, Vec2 value);
[[nodiscard]] Vec2 operator/(Vec2 value, float scalar);

struct Boid {
    Vec2 position;
    Vec2 velocity;
    float wander_angle = 0.0f;
    std::uint32_t random_state = 1;
    bool is_predator = false;
};

struct FlockSettings {
    int initial_flock_size = 1200;
    int initial_predator_count = 3;
    float boid_size = 6.0f;

    // Values are in pixels, seconds, and radians.
    float max_speed = 340.0f;
    float max_force = 450.0f;
    float alignment_radius = 78.0f;
    float cohesion_radius = 110.0f;
    float separation_radius = 22.0f;
    float alignment_weight = 0.72f;
    float cohesion_weight = 0.58f;
    float separation_weight = 1.35f;
    float wander_weight = 0.22f;
    float wander_jitter = 1.5f;
    // Bounds local candidate checks per prey update. Zero disables the cap.
    int max_sampled_prey_candidates = 96;

    // Predators hunt the closest prey they can see. Prey flee predators but
    // otherwise flock only with other prey.
    float predator_speed_multiplier = 1.4f;
    float predator_force_multiplier = 1.45f;
    float predator_perception_radius = 220.0f;
    float predator_pursuit_weight = 1.15f;
    float predator_separation_radius = 36.0f;
    float predator_separation_weight = 1.0f;
    float predator_wander_weight = 0.08f;
    float predator_avoidance_radius = 145.0f;
    float predator_avoidance_weight = 2.4f;

    // Soft wall steering begins 100 px from the physical edge; outward speed
    // extends that range so fast boids turn sooner. It deliberately remains
    // weaker than a full steering force; only the physical edge is hard.
    float wall_turn_distance = 100.0f;
    float wall_force_multiplier = 0.85f;
    float wall_braking_safety_factor = 2.5f;
    bool wrap_edges = false;
};

class FlockSimulation {
public:
    FlockSimulation(float width, float height, FlockSettings settings = {}, std::uint32_t seed = 1);

    void reset(int flock_size);
    // Replaces settings and recreates the flock so population and steering
    // changes take effect as one deterministic transition.
    void apply_settings(FlockSettings settings);
    void resize(float width, float height);
    void step(float seconds);

    [[nodiscard]] const std::vector<Boid>& boids() const;
    [[nodiscard]] const FlockSettings& settings() const;
    [[nodiscard]] float width() const;
    [[nodiscard]] float height() const;

private:
    float width_ = 1.0f;
    float height_ = 1.0f;
    FlockSettings settings_;
    std::uint32_t seed_ = 1;
    std::uint64_t step_index_ = 0;
    float cell_size_ = 1.0f;
    int grid_width_ = 1;
    int grid_height_ = 1;
    std::vector<Boid> boids_;
    std::vector<Boid> next_boids_;
    std::vector<std::vector<int>> grid_cells_;
    std::vector<int> predator_indices_;

    void rebuild_grid();
    void resize_grid();
    [[nodiscard]] Vec2 neighbor_offset(const Vec2& from, const Vec2& to) const;
    [[nodiscard]] Vec2 steer_toward(const Boid& boid, const Vec2& desired) const;
    [[nodiscard]] Vec2 wall_avoidance(const Boid& boid) const;
    [[nodiscard]] float minimum_wall_turn_distance() const;
    [[nodiscard]] float max_speed_for(const Boid& boid) const;
    [[nodiscard]] float max_force_for(const Boid& boid) const;
    [[nodiscard]] int cell_index_for(const Vec2& position) const;
    [[nodiscard]] Vec2 constrain_position(Vec2 position) const;
    void constrain_motion(Boid& boid) const;
};

}  // namespace boids
