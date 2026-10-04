#include "smartride/matching.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <random>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
namespace sr {
uint32_t nearest_driver(const std::vector<Driver>& drivers, Point p) {
    double best=inf; uint32_t result=invalid;
    for(const auto& d:drivers) if(d.available) { double c=distance(d.position,p); if(c<best || (c==best && d.id<result)) {best=c; result=d.id;} }
    return result;
}
std::vector<Driver> simulate_drivers(size_t n,const Graph& g,uint64_t seed) {
    if(g.points.empty() || n>=invalid) throw std::invalid_argument("driver simulation");
    std::mt19937_64 rng(seed); std::vector<Driver> v; v.reserve(n);
    for(size_t i=0;i<n;++i) v.push_back({uint32_t(i),g.points[rng()%g.points.size()],true}); return v;
}

SpatialGrid::SpatialGrid(const std::vector<Driver>& drivers, double cell_size)
    : drivers_(drivers) {
    if (drivers_.empty()) return;
    min_x_ = max_x_ = drivers_[0].position.x;
    min_y_ = max_y_ = drivers_[0].position.y;
    for (const auto& d : drivers_) {
        min_x_ = std::min(min_x_, d.position.x);
        max_x_ = std::max(max_x_, d.position.x);
        min_y_ = std::min(min_y_, d.position.y);
        max_y_ = std::max(max_y_, d.position.y);
    }
    if (cell_size > 0.0) {
        cell_size_ = cell_size;
    } else {
        double width = max_x_ - min_x_;
        double height = max_y_ - min_y_;
        double area = width * height;
        if (area > 0 && drivers_.size() > 0) {
            double target_cells = std::max(1.0, double(drivers_.size()) / 4.0);
            cell_size_ = std::max(50.0, std::sqrt(area / target_cells));
        } else {
            cell_size_ = 1000.0;
        }
    }
    if (!std::isfinite(cell_size_))
        throw std::invalid_argument("grid cell size exceeds supported range");
    auto dimension = [&](double span) {
        double count = std::floor(span / cell_size_) + 1;
        if (!std::isfinite(count) || count > std::numeric_limits<int>::max() / 2)
            throw std::invalid_argument("grid dimensions exceed supported range");
        return std::max(1, static_cast<int>(count));
    };
    cols_ = dimension(max_x_ - min_x_);
    rows_ = dimension(max_y_ - min_y_);
    cells_.resize(static_cast<size_t>(cols_) * rows_);
    for (size_t i = 0; i < drivers_.size(); ++i) {
        int cx = std::clamp(static_cast<int>(std::floor((drivers_[i].position.x - min_x_) / cell_size_)), 0, cols_ - 1);
        int cy = std::clamp(static_cast<int>(std::floor((drivers_[i].position.y - min_y_) / cell_size_)), 0, rows_ - 1);
        cells_[size_t(cy) * cols_ + cx].push_back(static_cast<uint32_t>(i));
    }
}

uint32_t SpatialGrid::nearest_driver(Point p) const {
    if (drivers_.empty()) return invalid;
    int cx = static_cast<int>(std::clamp(std::floor((p.x - min_x_) / cell_size_), 0.0, double(cols_ - 1)));
    int cy = static_cast<int>(std::clamp(std::floor((p.y - min_y_) / cell_size_), 0.0, double(rows_ - 1)));

    double best_dist = inf;
    uint32_t best_id = invalid;
    int max_r = std::max({cx, cols_ - 1 - cx, cy, rows_ - 1 - cy});

    for (int r = 0; r <= max_r; ++r) {
        int x_lo = cx - r;
        int x_hi = cx + r;
        int y_lo = cy - r;
        int y_hi = cy + r;

        auto check_cell = [&](int x, int y) {
            if (x < 0 || x >= cols_ || y < 0 || y >= rows_) return;
            for (uint32_t idx : cells_[size_t(y) * cols_ + x]) {
                const auto& d = drivers_[idx];
                if (!d.available) continue;
                double dist = distance(d.position, p);
                if (dist < best_dist || (dist == best_dist && d.id < best_id)) {
                    best_dist = dist;
                    best_id = d.id;
                }
            }
        };

        if (r == 0) {
            check_cell(cx, cy);
        } else {
            for (int x = x_lo; x <= x_hi; ++x) {
                check_cell(x, y_lo);
                check_cell(x, y_hi);
            }
            for (int y = y_lo + 1; y <= y_hi - 1; ++y) {
                check_cell(x_lo, y);
                check_cell(x_hi, y);
            }
        }

        int searched_min_x = std::max(0, x_lo);
        int searched_max_x = std::min(cols_ - 1, x_hi);
        int searched_min_y = std::max(0, y_lo);
        int searched_max_y = std::min(rows_ - 1, y_hi);

        if (searched_min_x == 0 && searched_max_x == cols_ - 1 &&
            searched_min_y == 0 && searched_max_y == rows_ - 1) {
            return best_id;
        }

        double min_unsearched = inf;
        if (searched_min_x > 0) {
            double bound = min_x_ + searched_min_x * cell_size_;
            min_unsearched = std::min(min_unsearched, p.x - bound);
        }
        if (searched_max_x < cols_ - 1) {
            double bound = min_x_ + (searched_max_x + 1) * cell_size_;
            min_unsearched = std::min(min_unsearched, bound - p.x);
        }
        if (searched_min_y > 0) {
            double bound = min_y_ + searched_min_y * cell_size_;
            min_unsearched = std::min(min_unsearched, p.y - bound);
        }
        if (searched_max_y < rows_ - 1) {
            double bound = min_y_ + (searched_max_y + 1) * cell_size_;
            min_unsearched = std::min(min_unsearched, bound - p.y);
        }

        if (best_id != invalid && best_dist < min_unsearched) {
            return best_id;
        }
    }
    return best_id;
}
namespace {
std::unordered_map<uint32_t, size_t> validate_batch(
    const std::vector<Driver>& drivers, const std::vector<RideRequest>& requests) {
    if (drivers.size() >= invalid) throw std::invalid_argument("too many drivers");
    std::unordered_map<uint32_t, size_t> indices;
    indices.reserve(drivers.size());
    auto finite = [](Point p) { return std::isfinite(p.x) && std::isfinite(p.y); };
    for (size_t i = 0; i < drivers.size(); ++i) {
        const auto& d = drivers[i];
        if (d.id == invalid || !finite(d.position) || !indices.emplace(d.id, i).second)
            throw std::invalid_argument("invalid driver ID or coordinates");
    }
    std::unordered_set<uint32_t> ids;
    ids.reserve(requests.size());
    for (const auto& r : requests)
        if (r.id == invalid || !finite(r.pickup) || !ids.insert(r.id).second)
            throw std::invalid_argument("invalid request ID or coordinates");
    return indices;
}

template<class Lookup>
std::vector<Assignment> assign_batch(std::vector<Driver>& drivers,
    const std::vector<RideRequest>& requests,
    const std::unordered_map<uint32_t, size_t>& indices, Lookup lookup) {
    std::vector<Assignment> result;
    result.reserve(requests.size());
    size_t remaining = std::count_if(drivers.begin(), drivers.end(),
        [](const Driver& d) { return d.available; });
    for (const auto& request : requests) {
        Assignment assignment{request.id};
        if (remaining) {
            assignment.driver_id = lookup(request.pickup);
            if (assignment.driver_id != invalid) {
                auto& driver = drivers[indices.at(assignment.driver_id)];
                assignment.pickup_distance = distance(driver.position, request.pickup);
                driver.available = false;
                --remaining;
            }
        }
        result.push_back(assignment);
    }
    return result;
}

void validate_hungarian_options(HungarianOptions options) {
    if (options.max_requests == 0 ||
        options.max_requests > kMaxHungarianRequests ||
        options.max_available_drivers == 0 ||
        options.max_available_drivers > kMaxHungarianAvailableDrivers)
        throw std::invalid_argument("invalid hungarian options");
}

// Rectangular Kuhn-Munkres for min-cost assignment with rows <= cols.
// cost[i * cols + j] is the cost of row i to column j. Equal reduced costs keep
// the earlier column (lower driver ID after sorting).
std::vector<size_t> kuhn_munkres(size_t rows, size_t cols, const std::vector<double>& cost) {
    std::vector<double> u(rows + 1), v(cols + 1);
    std::vector<int> p(cols + 1), way(cols + 1);
    for (size_t i = 1; i <= rows; ++i) {
        p[0] = static_cast<int>(i);
        int j0 = 0;
        std::vector<double> minv(cols + 1, inf);
        std::vector<char> used(cols + 1, 0);
        do {
            used[static_cast<size_t>(j0)] = 1;
            const int i0 = p[static_cast<size_t>(j0)];
            double delta = inf;
            int j1 = 0;
            for (size_t j = 1; j <= cols; ++j) {
                if (used[j]) continue;
                const double cur = cost[static_cast<size_t>(i0 - 1) * cols + (j - 1)]
                    - u[static_cast<size_t>(i0)] - v[j];
                if (cur < minv[j]) {
                    minv[j] = cur;
                    way[j] = j0;
                }
                if (minv[j] < delta) {
                    delta = minv[j];
                    j1 = static_cast<int>(j);
                }
            }
            for (size_t j = 0; j <= cols; ++j) {
                if (used[j]) {
                    u[static_cast<size_t>(p[j])] += delta;
                    v[j] -= delta;
                } else {
                    minv[j] -= delta;
                }
            }
            j0 = j1;
        } while (p[static_cast<size_t>(j0)] != 0);
        do {
            const int j1 = way[static_cast<size_t>(j0)];
            p[static_cast<size_t>(j0)] = p[static_cast<size_t>(j1)];
            j0 = j1;
        } while (j0);
    }
    std::vector<size_t> col_of_row(rows, static_cast<size_t>(-1));
    for (size_t j = 1; j <= cols; ++j)
        if (p[j] != 0)
            col_of_row[static_cast<size_t>(p[j] - 1)] = j - 1;
    return col_of_row;
}

std::vector<size_t> min_cost_assignment(size_t rows, size_t cols, const std::vector<double>& cost) {
    if (rows == 0 || cols == 0)
        return std::vector<size_t>(rows, static_cast<size_t>(-1));
    if (rows <= cols)
        return kuhn_munkres(rows, cols, cost);
    std::vector<double> transposed(cols * rows);
    for (size_t i = 0; i < rows; ++i)
        for (size_t j = 0; j < cols; ++j)
            transposed[j * rows + i] = cost[i * cols + j];
    const auto row_of_col = kuhn_munkres(cols, rows, transposed);
    std::vector<size_t> col_of_row(rows, static_cast<size_t>(-1));
    for (size_t j = 0; j < cols; ++j)
        if (row_of_col[j] != static_cast<size_t>(-1))
            col_of_row[row_of_col[j]] = j;
    return col_of_row;
}
}

std::vector<Assignment> greedy_batch_brute_force(std::vector<Driver>& drivers,
    const std::vector<RideRequest>& requests) {
    const auto indices = validate_batch(drivers, requests);
    return assign_batch(drivers, requests, indices,
        [&](Point p) { return nearest_driver(drivers, p); });
}

std::vector<Assignment> greedy_batch_spatial(std::vector<Driver>& drivers,
    const std::vector<RideRequest>& requests, double cell_size) {
    const auto indices = validate_batch(drivers, requests);
    if (!std::isfinite(cell_size) || cell_size < 0)
        throw std::invalid_argument("cell size must be finite and nonnegative");
    SpatialGrid grid(drivers, cell_size);
    return assign_batch(drivers, requests, indices,
        [&](Point p) { return grid.nearest_driver(p); });
}

std::vector<Assignment> hungarian_batch(std::vector<Driver>& drivers,
    const std::vector<RideRequest>& requests, HungarianOptions options) {
    [[maybe_unused]] const auto indices = validate_batch(drivers, requests);
    validate_hungarian_options(options);
    if (requests.size() > options.max_requests)
        throw std::invalid_argument("hungarian request limit exceeded");
    std::vector<size_t> available;
    available.reserve(drivers.size());
    for (size_t i = 0; i < drivers.size(); ++i)
        if (drivers[i].available) available.push_back(i);
    if (available.size() > options.max_available_drivers)
        throw std::invalid_argument("hungarian available driver limit exceeded");

    std::vector<Assignment> result;
    result.reserve(requests.size());
    for (const auto& request : requests)
        result.push_back(Assignment{request.id});
    if (requests.empty() || available.empty())
        return result;

    std::sort(available.begin(), available.end(),
        [&](size_t a, size_t b) { return drivers[a].id < drivers[b].id; });
    const size_t rows = requests.size();
    const size_t cols = available.size();
    std::vector<double> cost(rows * cols);
    for (size_t i = 0; i < rows; ++i)
        for (size_t j = 0; j < cols; ++j)
            cost[i * cols + j] = distance(drivers[available[j]].position, requests[i].pickup);

    const auto col_of_row = min_cost_assignment(rows, cols, cost);
    std::vector<size_t> winners;
    winners.reserve(rows);
    for (size_t i = 0; i < rows; ++i) {
        if (col_of_row[i] == static_cast<size_t>(-1)) continue;
        result[i].driver_id = drivers[available[col_of_row[i]]].id;
        result[i].pickup_distance = cost[i * cols + col_of_row[i]];
        winners.push_back(available[col_of_row[i]]);
    }
    for (size_t idx : winners)
        drivers[idx].available = false;
    return result;
}

}
