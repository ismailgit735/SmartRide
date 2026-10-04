#include "smartride/matching.hpp"
#include <algorithm>
#include <cmath>
#include <random>
#include <stdexcept>
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
    cols_ = std::max(1, static_cast<int>(std::floor((max_x_ - min_x_) / cell_size_)) + 1);
    rows_ = std::max(1, static_cast<int>(std::floor((max_y_ - min_y_) / cell_size_)) + 1);
    cells_.resize(static_cast<size_t>(cols_) * rows_);
    for (size_t i = 0; i < drivers_.size(); ++i) {
        int cx = std::clamp(static_cast<int>(std::floor((drivers_[i].position.x - min_x_) / cell_size_)), 0, cols_ - 1);
        int cy = std::clamp(static_cast<int>(std::floor((drivers_[i].position.y - min_y_) / cell_size_)), 0, rows_ - 1);
        cells_[cy * cols_ + cx].push_back(static_cast<uint32_t>(i));
    }
}

uint32_t SpatialGrid::nearest_driver(Point p) const {
    if (drivers_.empty()) return invalid;
    int cx = std::clamp(static_cast<int>(std::floor((p.x - min_x_) / cell_size_)), 0, cols_ - 1);
    int cy = std::clamp(static_cast<int>(std::floor((p.y - min_y_) / cell_size_)), 0, rows_ - 1);

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
            for (uint32_t idx : cells_[y * cols_ + x]) {
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
}
