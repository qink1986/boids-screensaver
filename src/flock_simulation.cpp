#include "flock_simulation.h"

#include <algorithm>
#include <array>
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
// The grid cell size is at least the largest prey flocking radius, so every
// prey query touches at most a 3x3 neighborhood.
constexpr int kPreyCellRange = 1;
constexpr int kMaxPreyNeighborCells = 9;

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

std::uint32_t mix_random(std::uint32_t value) {
    value ^= value >> 16;
    value *= 0x7FEB352Du;
    value ^= value >> 15;
    value *= 0x846CA68Bu;
    value ^= value >> 16;
    return value;
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
    const int predator_count = std::max(settings_.initial_predator_count, 0);
    const int total_boid_count = flock_size + predator_count;
    boids_.clear();
    boids_.reserve(static_cast<std::size_t>(total_boid_count));

    std::uint32_t random_state = seed_;
    for (int index = 0; index < total_boid_count; ++index) {
        Boid boid;
        boid.is_predator = index >= flock_size;
        boid.position = constrain_position(
            {random_unit(random_state) * width_, random_unit(random_state) * height_});
        boid.random_state = next_random(random_state) | 1u;
        boid.velocity = random_direction(boid.random_state) * max_speed_for(boid);
        boid.wander_angle = random_unit(boid.random_state) * 2.0f * kPi;
        boids_.push_back(boid);
    }

    next_boids_.resize(boids_.size());
    step_index_ = 0;
    resize_grid();
}

void FlockSimulation::apply_settings(FlockSettings settings) {
    settings_ = settings;
    reset(settings_.initial_flock_size);
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
    // Prey only need a grid for their three flocking radii. Predator behavior
    // is calculated against the small predator list or by three exact hunts.
    cell_size_ = std::max({
        settings_.alignment_radius,
        settings_.cohesion_radius,
        settings_.separation_radius,
        1.0f,
    });
    grid_width_ = std::max(1, static_cast<int>(std::ceil(width_ / cell_size_)));
    grid_height_ = std::max(1, static_cast<int>(std::ceil(height_ / cell_size_)));
    grid_cells_.clear();
    grid_cells_.resize(static_cast<std::size_t>(grid_width_ * grid_height_));
    predator_indices_.clear();
}

void FlockSimulation::rebuild_grid() {
    const std::size_t cell_count = static_cast<std::size_t>(grid_width_ * grid_height_);
    if (grid_cells_.size() != cell_count) {
        resize_grid();
    }
    for (std::vector<int>& cell : grid_cells_) {
        cell.clear();
    }
    predator_indices_.clear();

    for (std::size_t index = 0; index < boids_.size(); ++index) {
        if (boids_[index].is_predator) {
            predator_indices_.push_back(static_cast<int>(index));
        } else {
            grid_cells_[cell_index_for(boids_[index].position)].push_back(static_cast<int>(index));
        }
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

float FlockSimulation::max_speed_for(const Boid& boid) const {
    return settings_.max_speed * (boid.is_predator ? settings_.predator_speed_multiplier : 1.0f);
}

float FlockSimulation::max_force_for(const Boid& boid) const {
    return settings_.max_force * (boid.is_predator ? settings_.predator_force_multiplier : 1.0f);
}

Vec2 FlockSimulation::steer_toward(const Boid& boid, const Vec2& desired) const {
    if (desired.length_squared() <= kEpsilon) {
        return {};
    }
    return (desired.normalized() * max_speed_for(boid) - boid.velocity).limited(max_force_for(boid));
}

float FlockSimulation::minimum_wall_turn_distance() const {
    return std::max(settings_.wall_turn_distance, 1.0f);
}

Vec2 FlockSimulation::wall_avoidance(const Boid& boid) const {
    if (settings_.wrap_edges) {
        return {};
    }

    const float minimum_turn_distance = minimum_wall_turn_distance();
    const float braking_force = std::max(max_force_for(boid), kEpsilon);
    const float maximum_wall_force = max_force_for(boid) *
        clamp(settings_.wall_force_multiplier, 0.0f, 1.0f);
    const float safety_factor = std::max(settings_.wall_braking_safety_factor, 0.0f);
    Vec2 steering;

    const auto add_wall_force = [&steering, minimum_turn_distance, braking_force, safety_factor](
                                    float distance, float outward_speed, float inward_x, float inward_y) {
        const float outward_speed_for_braking = std::max(outward_speed, 0.0f);
        const float braking_distance = outward_speed_for_braking * outward_speed_for_braking /
            (2.0f * braking_force);
        const float turn_distance = minimum_turn_distance + braking_distance * safety_factor;
        if (distance >= turn_distance) {
            return;
        }

        // Soft wall: closer position means stronger inward force. Outward
        // velocity only enlarges the turn zone; it never disables wall force.
        const float proximity = clamp(1.0f - distance / turn_distance, 0.0f, 1.0f);
        // Cubic curve keeps the outer soft-wall region gentle, then increases
        // steadily closer to the physical edge.
        const float influence = proximity * proximity * proximity;
        steering.x += inward_x * influence;
        steering.y += inward_y * influence;
    };

    add_wall_force(boid.position.x, -boid.velocity.x, 1.0f, 0.0f);
    add_wall_force(width_ - boid.position.x, boid.velocity.x, -1.0f, 0.0f);
    add_wall_force(boid.position.y, -boid.velocity.y, 0.0f, 1.0f);
    add_wall_force(height_ - boid.position.y, boid.velocity.y, 0.0f, -1.0f);

    if (steering.length_squared() <= kEpsilon) {
        return {};
    }
    return steering.limited(1.0f) * maximum_wall_force;
}

Vec2 FlockSimulation::constrain_position(Vec2 position) const {
    if (settings_.wrap_edges) {
        position.x = wrap(position.x, width_);
        position.y = wrap(position.y, height_);
    } else {
        position.x = clamp(position.x, 0.0f, std::nextafter(width_, 0.0f));
        position.y = clamp(position.y, 0.0f, std::nextafter(height_, 0.0f));
    }
    return position;
}

void FlockSimulation::constrain_motion(Boid& boid) const {
    if (settings_.wrap_edges) {
        return;
    }

    const float maximum_x = std::nextafter(width_, 0.0f);
    const float maximum_y = std::nextafter(height_, 0.0f);
    if (boid.position.x <= 0.0f && boid.velocity.x < 0.0f) {
        boid.velocity.x = -boid.velocity.x;
    } else if (boid.position.x >= maximum_x && boid.velocity.x > 0.0f) {
        boid.velocity.x = -boid.velocity.x;
    }
    if (boid.position.y <= 0.0f && boid.velocity.y < 0.0f) {
        boid.velocity.y = -boid.velocity.y;
    } else if (boid.position.y >= maximum_y && boid.velocity.y > 0.0f) {
        boid.velocity.y = -boid.velocity.y;
    }
}

void FlockSimulation::step(float seconds) {
    if (boids_.empty()) {
        return;
    }

    const float dt = clamp(seconds, kMinimumStepSeconds, kMaximumStepSeconds);
    const float prey_query_radius = std::max({
        settings_.alignment_radius,
        settings_.cohesion_radius,
        settings_.separation_radius,
    });
    const float prey_query_radius_squared = prey_query_radius * prey_query_radius;
    const float alignment_radius_squared = settings_.alignment_radius * settings_.alignment_radius;
    const float cohesion_radius_squared = settings_.cohesion_radius * settings_.cohesion_radius;
    const float separation_radius_squared = settings_.separation_radius * settings_.separation_radius;
    const float predator_perception_radius_squared =
        settings_.predator_perception_radius * settings_.predator_perception_radius;
    const float predator_separation_radius_squared =
        settings_.predator_separation_radius * settings_.predator_separation_radius;
    const float predator_avoidance_radius_squared =
        settings_.predator_avoidance_radius * settings_.predator_avoidance_radius;
    const int cell_range = kPreyCellRange;
    const int max_sampled_candidates = std::max(settings_.max_sampled_prey_candidates, 0);

    rebuild_grid();

#ifdef _OPENMP
#pragma omp parallel for if (boids_.size() >= 256)
#endif
    for (int index = 0; index < static_cast<int>(boids_.size()); ++index) {
        const Boid& boid = boids_[static_cast<std::size_t>(index)];
        Vec2 average_velocity;
        Vec2 average_offset;
        Vec2 separation;
        Vec2 predator_avoidance;
        Vec2 closest_prey_offset;
        float closest_prey_distance_squared = std::numeric_limits<float>::max();
        int alignment_count = 0;
        int cohesion_count = 0;
        int separation_count = 0;
        int predator_separation_count = 0;
        int predator_avoidance_count = 0;

        if (boid.is_predator) {
            // There are only a few predators. Exact prey scans keep pursuit
            // behavior stable without making dense prey flocks expensive.
            for (std::size_t neighbor_index = 0; neighbor_index < boids_.size(); ++neighbor_index) {
                const Boid& neighbor = boids_[neighbor_index];
                if (neighbor.is_predator) {
                    continue;
                }
                const Vec2 offset = neighbor_offset(boid.position, neighbor.position);
                const float distance_squared = offset.length_squared();
                if (distance_squared > kEpsilon &&
                    distance_squared <= predator_perception_radius_squared &&
                    distance_squared < closest_prey_distance_squared) {
                    closest_prey_offset = offset;
                    closest_prey_distance_squared = distance_squared;
                }
            }
            for (const int other_predator_index : predator_indices_) {
                if (other_predator_index == index) {
                    continue;
                }
                const Vec2 offset = neighbor_offset(
                    boid.position, boids_[static_cast<std::size_t>(other_predator_index)].position);
                const float distance_squared = offset.length_squared();
                if (distance_squared > kEpsilon &&
                    distance_squared <= predator_separation_radius_squared) {
                    separation -= offset / distance_squared;
                    ++predator_separation_count;
                }
            }
        } else {
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
            const int scanned_cell_count = cells_to_scan_x * cells_to_scan_y;
            const std::uint32_t sampling_seed = mix_random(
                boid.random_state ^ static_cast<std::uint32_t>(step_index_) ^ 0x9E3779B9u);
            const int first_cell_offset = static_cast<int>(sampling_seed % scanned_cell_count);
            std::array<const std::vector<int>*, kMaxPreyNeighborCells> cells{};
            std::array<std::size_t, kMaxPreyNeighborCells> cell_starts{};
            std::array<std::size_t, kMaxPreyNeighborCells> cell_offsets{};
            int nonempty_cell_count = 0;

            // Rotate cells and alternate among them. A dense cell therefore
            // cannot permanently crowd out neighbors from one side of a flock.
            for (int cell_order = 0; cell_order < scanned_cell_count; ++cell_order) {
                const int rotated_cell_offset = (first_cell_offset + cell_order) % scanned_cell_count;
                int scanned_cell_y = center_cell_y - cell_range +
                    rotated_cell_offset / cells_to_scan_x;
                int scanned_cell_x = center_cell_x - cell_range +
                    rotated_cell_offset % cells_to_scan_x;
                if (settings_.wrap_edges) {
                    scanned_cell_y = (scanned_cell_y % grid_height_ + grid_height_) % grid_height_;
                    scanned_cell_x = (scanned_cell_x % grid_width_ + grid_width_) % grid_width_;
                } else if (scanned_cell_y < 0 || scanned_cell_y >= grid_height_ ||
                           scanned_cell_x < 0 || scanned_cell_x >= grid_width_) {
                    continue;
                }

                const std::vector<int>& cell = grid_cells_[scanned_cell_y * grid_width_ + scanned_cell_x];
                if (cell.empty()) {
                    continue;
                }

                cells[nonempty_cell_count] = &cell;
                cell_starts[nonempty_cell_count] = cell.size() > 1
                    ? static_cast<std::size_t>(mix_random(
                        sampling_seed ^ static_cast<std::uint32_t>(rotated_cell_offset)) % cell.size())
                    : 0;
                ++nonempty_cell_count;
            }

            int sampled_candidate_count = 0;
            int active_cell_count = nonempty_cell_count;
            int current_cell = 0;
            while (active_cell_count > 0 &&
                   (max_sampled_candidates == 0 || sampled_candidate_count < max_sampled_candidates)) {
                const std::vector<int>& cell = *cells[current_cell];
                if (cell_offsets[current_cell] == cell.size()) {
                    --active_cell_count;
                    cells[current_cell] = cells[active_cell_count];
                    cell_starts[current_cell] = cell_starts[active_cell_count];
                    cell_offsets[current_cell] = cell_offsets[active_cell_count];
                    if (current_cell == active_cell_count) {
                        current_cell = 0;
                    }
                    continue;
                }

                const int neighbor_index = cell[(cell_starts[current_cell] + cell_offsets[current_cell]) %
                    cell.size()];
                ++cell_offsets[current_cell];
                if (neighbor_index != index) {
                    ++sampled_candidate_count;

                    const Boid& neighbor = boids_[static_cast<std::size_t>(neighbor_index)];
                    const Vec2 offset = neighbor_offset(boid.position, neighbor.position);
                    const float distance_squared = offset.length_squared();
                    if (distance_squared > kEpsilon && distance_squared <= prey_query_radius_squared) {
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

                current_cell = (current_cell + 1) % active_cell_count;
            }

            // Predator count is intentionally tiny. Exact checks prevent the
            // neighbor cap from hiding a nearby predator in a dense flock.
            for (const int predator_index : predator_indices_) {
                const Vec2 offset = neighbor_offset(
                    boid.position, boids_[static_cast<std::size_t>(predator_index)].position);
                const float distance_squared = offset.length_squared();
                if (distance_squared > kEpsilon &&
                    distance_squared <= predator_avoidance_radius_squared) {
                    predator_avoidance -= offset / distance_squared;
                    ++predator_avoidance_count;
                }
            }
        }

        Vec2 steering;
        if (boid.is_predator) {
            if (closest_prey_distance_squared < std::numeric_limits<float>::max()) {
                steering += steer_toward(boid, closest_prey_offset) * settings_.predator_pursuit_weight;
            }
            if (predator_separation_count > 0) {
                steering += steer_toward(
                                boid, separation / static_cast<float>(predator_separation_count)) *
                            settings_.predator_separation_weight;
            }
        } else {
            if (alignment_count > 0) {
                steering += steer_toward(boid, average_velocity / static_cast<float>(alignment_count)) *
                            settings_.alignment_weight;
            }
            if (cohesion_count > 0) {
                steering += steer_toward(boid, average_offset / static_cast<float>(cohesion_count)) *
                            settings_.cohesion_weight;
            }
            if (separation_count > 0) {
                steering += steer_toward(
                                boid, separation / static_cast<float>(separation_count)) *
                            settings_.separation_weight;
            }
            if (predator_avoidance_count > 0) {
                steering += steer_toward(
                                boid, predator_avoidance / static_cast<float>(predator_avoidance_count)) *
                            settings_.predator_avoidance_weight;
            }
        }

        Boid next = boid;
        next.wander_angle += random_signed(next.random_state) * settings_.wander_jitter * dt;
        const Vec2 forward = boid.velocity.normalized();
        const Vec2 wander_direction{
            std::cos(next.wander_angle) * 0.7f + forward.x,
            std::sin(next.wander_angle) * 0.7f + forward.y,
        };
        const float wander_weight = boid.is_predator
            ? settings_.predator_wander_weight
            : settings_.wander_weight;
        steering += steer_toward(boid, wander_direction) * wander_weight;

        const Vec2 wall_steering = wall_avoidance(boid);
        const float wall_priority = clamp(
            wall_steering.length() / std::max(max_force_for(boid), kEpsilon), 0.0f, 1.0f);
        // Close to a wall, suppress flocking, pursuit, and wander forces so
        // inward steering wins. Farther away, behavior blends smoothly.
        steering *= 1.0f - wall_priority;
        steering += wall_steering;
        steering = steering.limited(max_force_for(boid));

        next.velocity = (boid.velocity + steering * dt).limited(max_speed_for(boid));
        next.position = constrain_position(boid.position + next.velocity * dt);
        constrain_motion(next);
        next_boids_[static_cast<std::size_t>(index)] = next;
    }

    boids_.swap(next_boids_);
    ++step_index_;
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
