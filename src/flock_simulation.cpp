#include "flock_simulation.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

#ifdef _OPENMP
#include <omp.h>
#endif

namespace boids {
namespace {

constexpr float kEpsilon = 1.0e-6f;
constexpr float kMinimumStepSeconds = 1.0e-5f;
constexpr float kMaximumStepSeconds = 1.0f / 30.0f;
constexpr float kPi = 3.14159265358979323846f;

float clamp(float value, float minimum, float maximum) {
    return std::clamp(value, minimum, maximum);
}

float wrap(float value, float maximum) {
    value = std::fmod(value, maximum);
    return value < 0.0f ? value + maximum : value;
}

std::uint32_t next_random(std::uint32_t& state) {
    state = state * 1664525u + 1013904223u;
    return state;
}

float random_unit(std::uint32_t& state) {
    return static_cast<float>(next_random(state) >> 8) * (1.0f / 16777216.0f);
}

float random_signed(std::uint32_t& state) {
    return random_unit(state) * 2.0f - 1.0f;
}

Vec2 random_direction(std::uint32_t& state) {
    const float angle = random_unit(state) * 2.0f * kPi;
    return {std::cos(angle), std::sin(angle)};
}

}  // namespace

Vec2& Vec2::operator+=(const Vec2& other) {
    x += other.x;
    y += other.y;
    return *this;
}

Vec2& Vec2::operator-=(const Vec2& other) {
    x -= other.x;
    y -= other.y;
    return *this;
}

Vec2& Vec2::operator*=(float scalar) {
    x *= scalar;
    y *= scalar;
    return *this;
}

float Vec2::length_squared() const {
    return x * x + y * y;
}

float Vec2::length() const {
    return std::sqrt(length_squared());
}

Vec2 Vec2::normalized() const {
    const float magnitude = length();
    return magnitude > kEpsilon ? *this / magnitude : Vec2{};
}

Vec2 Vec2::limited(float maximum) const {
    const float maximum_squared = maximum * maximum;
    const float magnitude_squared = length_squared();
    return magnitude_squared > maximum_squared ? normalized() * maximum : *this;
}

Vec2 operator+(Vec2 left, const Vec2& right) {
    return left += right;
}

Vec2 operator-(Vec2 left, const Vec2& right) {
    return left -= right;
}

Vec2 operator*(Vec2 value, float scalar) {
    return value *= scalar;
}

Vec2 operator*(float scalar, Vec2 value) {
    return value *= scalar;
}

Vec2 operator/(Vec2 value, float scalar) {
    return value *= 1.0f / scalar;
}

FlockSimulation::FlockSimulation(float width, float height, FlockSettings settings, std::uint32_t seed)
    : width_(std::max(width, 1.0f)),
      height_(std::max(height, 1.0f)),
      settings_(settings),
      seed_(seed == 0 ? 1 : seed) {
    reset(settings_.initial_flock_size);
}

void FlockSimulation::reset(int flock_size) {
    flock_size = std::max(flock_size, 0);
    boids_.clear();
    boids_.reserve(static_cast<std::size_t>(flock_size));

    std::uint32_t random_state = seed_;
    for (int index = 0; index < flock_size; ++index) {
        Boid boid;
        boid.position = {random_unit(random_state) * width_, random_unit(random_state) * height_};
        boid.random_state = next_random(random_state) | 1u;
        boid.velocity = random_direction(boid.random_state) * settings_.max_speed;
        boid.wander_angle = random_unit(boid.random_state) * 2.0f * kPi;
        boids_.push_back(boid);
    }

    next_boids_.resize(boids_.size());
    resize_grid();
}

void FlockSimulation::resize(float width, float height) {
    width_ = std::max(width, 1.0f);
    height_ = std::max(height, 1.0f);
    for (Boid& boid : boids_) {
        boid.position = constrain_position(boid.position);
    }
    resize_grid();
}

void FlockSimulation::resize_grid() {
    cell_size_ = std::max(
        {settings_.alignment_radius, settings_.cohesion_radius, settings_.separation_radius, 1.0f});
    grid_width_ = std::max(1, static_cast<int>(std::ceil(width_ / cell_size_)));
    grid_height_ = std::max(1, static_cast<int>(std::ceil(height_ / cell_size_)));
    grid_heads_.assign(static_cast<std::size_t>(grid_width_ * grid_height_), -1);
    grid_links_.assign(boids_.size(), -1);
}

void FlockSimulation::rebuild_grid() {
    if (grid_links_.size() != boids_.size()) {
        resize_grid();
    }
    std::fill(grid_heads_.begin(), grid_heads_.end(), -1);
    std::fill(grid_links_.begin(), grid_links_.end(), -1);

    for (std::size_t index = 0; index < boids_.size(); ++index) {
        const int cell_index = cell_index_for(boids_[index].position);
        grid_links_[index] = grid_heads_[cell_index];
        grid_heads_[cell_index] = static_cast<int>(index);
    }
}

int FlockSimulation::cell_index_for(const Vec2& position) const {
    int cell_x = static_cast<int>(position.x / cell_size_);
    int cell_y = static_cast<int>(position.y / cell_size_);
    cell_x = std::clamp(cell_x, 0, grid_width_ - 1);
    cell_y = std::clamp(cell_y, 0, grid_height_ - 1);
    return cell_y * grid_width_ + cell_x;
}

Vec2 FlockSimulation::neighbor_offset(const Vec2& from, const Vec2& to) const {
    Vec2 offset = to - from;
    if (!settings_.wrap_edges) {
        return offset;
    }

    if (offset.x > width_ * 0.5f) {
        offset.x -= width_;
    } else if (offset.x < -width_ * 0.5f) {
        offset.x += width_;
    }
    if (offset.y > height_ * 0.5f) {
        offset.y -= height_;
    } else if (offset.y < -height_ * 0.5f) {
        offset.y += height_;
    }
    return offset;
}

Vec2 FlockSimulation::steer_toward(const Vec2& velocity, const Vec2& desired) const {
    if (desired.length_squared() <= kEpsilon) {
        return {};
    }
    return (desired.normalized() * settings_.max_speed - velocity).limited(settings_.max_force);
}

Vec2 FlockSimulation::constrain_position(Vec2 position) const {
    if (settings_.wrap_edges) {
        position.x = wrap(position.x, width_);
        position.y = wrap(position.y, height_);
    } else {
        position.x = clamp(position.x, 0.0f, width_);
        position.y = clamp(position.y, 0.0f, height_);
    }
    return position;
}

void FlockSimulation::step(float seconds) {
    if (boids_.empty()) {
        return;
    }

    const float dt = clamp(seconds, kMinimumStepSeconds, kMaximumStepSeconds);
    const float query_radius = std::max(
        {settings_.alignment_radius, settings_.cohesion_radius, settings_.separation_radius});
    const float query_radius_squared = query_radius * query_radius;
    const float alignment_radius_squared = settings_.alignment_radius * settings_.alignment_radius;
    const float cohesion_radius_squared = settings_.cohesion_radius * settings_.cohesion_radius;
    const float separation_radius_squared = settings_.separation_radius * settings_.separation_radius;
    const int cell_range = std::max(1, static_cast<int>(std::ceil(query_radius / cell_size_)));

    rebuild_grid();

#ifdef _OPENMP
#pragma omp parallel for if (boids_.size() >= 256)
#endif
    for (int index = 0; index < static_cast<int>(boids_.size()); ++index) {
        const Boid& boid = boids_[static_cast<std::size_t>(index)];
        Vec2 average_velocity;
        Vec2 average_offset;
        Vec2 separation;
        int alignment_count = 0;
        int cohesion_count = 0;
        int separation_count = 0;

        const int center_cell_x = std::clamp(
            static_cast<int>(boid.position.x / cell_size_), 0, grid_width_ - 1);
        const int center_cell_y = std::clamp(
            static_cast<int>(boid.position.y / cell_size_), 0, grid_height_ - 1);

        const int cells_to_scan_y = settings_.wrap_edges
            ? std::min(grid_height_, 2 * cell_range + 1)
            : 2 * cell_range + 1;
        const int cells_to_scan_x = settings_.wrap_edges
            ? std::min(grid_width_, 2 * cell_range + 1)
            : 2 * cell_range + 1;

        for (int offset_y = 0; offset_y < cells_to_scan_y; ++offset_y) {
            int scanned_cell_y = center_cell_y - cell_range + offset_y;
            if (settings_.wrap_edges) {
                scanned_cell_y = (scanned_cell_y % grid_height_ + grid_height_) % grid_height_;
            } else if (scanned_cell_y < 0 || scanned_cell_y >= grid_height_) {
                continue;
            }

            for (int offset_x = 0; offset_x < cells_to_scan_x; ++offset_x) {
                int scanned_cell_x = center_cell_x - cell_range + offset_x;
                if (settings_.wrap_edges) {
                    scanned_cell_x = (scanned_cell_x % grid_width_ + grid_width_) % grid_width_;
                } else if (scanned_cell_x < 0 || scanned_cell_x >= grid_width_) {
                    continue;
                }

                int neighbor_index = grid_heads_[scanned_cell_y * grid_width_ + scanned_cell_x];
                while (neighbor_index != -1) {
                    if (neighbor_index != index) {
                        const Boid& neighbor = boids_[static_cast<std::size_t>(neighbor_index)];
                        const Vec2 offset = neighbor_offset(boid.position, neighbor.position);
                        const float distance_squared = offset.length_squared();
                        if (distance_squared > kEpsilon && distance_squared <= query_radius_squared) {
                            if (distance_squared <= alignment_radius_squared) {
                                average_velocity += neighbor.velocity;
                                ++alignment_count;
                            }
                            if (distance_squared <= cohesion_radius_squared) {
                                average_offset += offset;
                                ++cohesion_count;
                            }
                            if (distance_squared <= separation_radius_squared) {
                                separation -= offset / distance_squared;
                                ++separation_count;
                            }
                        }
                    }
                    neighbor_index = grid_links_[static_cast<std::size_t>(neighbor_index)];
                }
            }
        }

        Vec2 steering;
        if (alignment_count > 0) {
            steering += steer_toward(boid.velocity, average_velocity / static_cast<float>(alignment_count)) *
                        settings_.alignment_weight;
        }
        if (cohesion_count > 0) {
            steering += steer_toward(boid.velocity, average_offset / static_cast<float>(cohesion_count)) *
                        settings_.cohesion_weight;
        }
        if (separation_count > 0) {
            steering += steer_toward(
                            boid.velocity, separation / static_cast<float>(separation_count)) *
                        settings_.separation_weight;
        }

        Boid next = boid;
        next.wander_angle += random_signed(next.random_state) * settings_.wander_jitter * dt;
        const Vec2 forward = boid.velocity.normalized();
        const Vec2 wander_direction{
            std::cos(next.wander_angle) * 0.7f + forward.x,
            std::sin(next.wander_angle) * 0.7f + forward.y,
        };
        steering += steer_toward(boid.velocity, wander_direction) * settings_.wander_weight;
        steering = steering.limited(settings_.max_force);

        next.velocity = (boid.velocity + steering * dt).limited(settings_.max_speed);
        next.position = constrain_position(boid.position + next.velocity * dt);
        next_boids_[static_cast<std::size_t>(index)] = next;
    }

    boids_.swap(next_boids_);
}

const std::vector<Boid>& FlockSimulation::boids() const {
    return boids_;
}

const FlockSettings& FlockSimulation::settings() const {
    return settings_;
}

float FlockSimulation::width() const {
    return width_;
}

float FlockSimulation::height() const {
    return height_;
}

}  // namespace boids
