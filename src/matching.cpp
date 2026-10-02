#include "smartride/matching.hpp"
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
}
