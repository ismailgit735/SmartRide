#pragma once
#include "graph.hpp"
namespace sr {
struct Driver { uint32_t id; Point position; bool available=true; };
uint32_t nearest_driver(const std::vector<Driver>& drivers, Point request);
std::vector<Driver> simulate_drivers(size_t count, const Graph& g, uint64_t seed);

class SpatialGrid {
    const std::vector<Driver>& drivers_;
    double min_x_ = 0, min_y_ = 0;
    double max_x_ = 0, max_y_ = 0;
    double cell_size_ = 1000.0;
    int cols_ = 0, rows_ = 0;
    std::vector<std::vector<uint32_t>> cells_;
public:
    explicit SpatialGrid(const std::vector<Driver>& drivers, double cell_size = 0.0);
    uint32_t nearest_driver(Point request) const;
    size_t cols() const { return cols_; }
    size_t rows() const { return rows_; }
    double cell_size() const { return cell_size_; }
};
}
