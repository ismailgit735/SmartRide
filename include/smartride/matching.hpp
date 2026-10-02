#pragma once
#include "graph.hpp"
namespace sr {
struct Driver { uint32_t id; Point position; bool available=true; };
uint32_t nearest_driver(const std::vector<Driver>& drivers, Point request);
std::vector<Driver> simulate_drivers(size_t count, const Graph& g, uint64_t seed);
}
