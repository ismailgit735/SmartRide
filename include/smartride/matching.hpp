#pragma once
#include "graph.hpp"
#include <cstddef>
namespace sr {
struct Driver { uint32_t id; Point position; bool available=true; };
uint32_t nearest_driver(const std::vector<Driver>& drivers, Point request);
std::vector<Driver> simulate_drivers(size_t count, const Graph& g, uint64_t seed);

struct RideRequest { uint32_t id; Point pickup; };
struct Assignment {
    uint32_t request_id;
    uint32_t driver_id = invalid;
    double pickup_distance = inf;
};
// Process requests in supplied order, reserving winners immediately. IDs must
// be unique within each input and not invalid; coordinates must be finite.
// Validation completes before availability changes. Unmatched: invalid/inf.
// Distances are Euclidean; these APIs are single-threaded.
std::vector<Assignment> greedy_batch_brute_force(
    std::vector<Driver>& drivers, const std::vector<RideRequest>& requests);
// Builds one grid per call; cell_size=0 selects the existing automatic sizing.
// Explicit sizes must be positive and finite; unsupported grid dimensions throw
// before mutation. Memory allocation failures during setup also precede mutation.
std::vector<Assignment> greedy_batch_spatial(
    std::vector<Driver>& drivers, const std::vector<RideRequest>& requests,
    double cell_size = 0.0);

constexpr size_t kMaxHungarianRequests = 64;
constexpr size_t kMaxHungarianAvailableDrivers = 512;
struct HungarianOptions {
    size_t max_requests = kMaxHungarianRequests;
    size_t max_available_drivers = kMaxHungarianAvailableDrivers;
};
// Exact min-cost assignment of the supplied batch: max matched cardinality,
// then min total Euclidean pickup. Available drivers are sorted by ID; request
// input order is the row order. Hard limits are 64 requests and 512 available
// drivers; options may only tighten those caps. Oversized batches throw before
// mutation. The complete matching is computed before any reservation. Unmatched
// results use invalid/inf. No greedy fallback. Single-threaded; Euclidean cost.
std::vector<Assignment> hungarian_batch(
    std::vector<Driver>& drivers, const std::vector<RideRequest>& requests,
    HungarianOptions options = {});

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
