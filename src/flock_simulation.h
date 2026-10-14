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
};

struct FlockSettings {
    int initial_flock_size = 1200;
    float boid_size = 6.0f;

    // Values are in pixels, seconds, and radians.
    float max_speed = 250.0f;
    float max_force = 450.0f;
    float alignment_radius = 78.0f;
    float cohesion_radius = 110.0f;
    float separation_radius = 22.0f;
    float alignment_weight = 0.72f;
    float cohesion_weight = 0.58f;
    float separation_weight = 1.35f;
    float wander_weight = 0.22f;
    float wander_jitter = 1.5f;

    bool wrap_edges = true;
};

class FlockSimulation {
public:
    FlockSimulation(float width, float height, FlockSettings settings = {}, std::uint32_t seed = 1);

    void reset(int flock_size);
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
    float cell_size_ = 1.0f;
    int grid_width_ = 1;
    int grid_height_ = 1;
    std::vector<Boid> boids_;
    std::vector<Boid> next_boids_;
    std::vector<int> grid_heads_;
    std::vector<int> grid_links_;

    void rebuild_grid();
    void resize_grid();
    [[nodiscard]] Vec2 neighbor_offset(const Vec2& from, const Vec2& to) const;
    [[nodiscard]] Vec2 steer_toward(const Vec2& velocity, const Vec2& desired) const;
    [[nodiscard]] int cell_index_for(const Vec2& position) const;
    [[nodiscard]] Vec2 constrain_position(Vec2 position) const;
};

}  // namespace boids
