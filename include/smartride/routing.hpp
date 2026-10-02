#pragma once
#include "graph.hpp"
namespace sr {
struct Route { double distance=inf; std::vector<EdgeId> edges; uint64_t settled=0; };
Route dijkstra(const Graph& g, Node s, Node t);
bool valid_route(const Graph& g, Node s, Node t, const Route& r);
}

namespace sr {
// Graph must remain unchanged for this object's lifetime. Queries are thread-safe.
class Router {
    const Graph& g_;
    double scale_=1;
public:
    explicit Router(const Graph& g);
    double heuristic_scale() const { return scale_; }
    Route astar(Node s, Node t) const;
    Route bidirectional_astar(Node s, Node t) const;
};
class ContractionHierarchy {
    struct Arc { Node from,to; double weight; EdgeId left=invalid,right=invalid,original=invalid; };
    const Graph& g_;
    std::vector<Arc> arcs_;
    std::vector<std::vector<EdgeId>> out_,in_;
    std::vector<uint32_t> rank_;
    void unpack(EdgeId id, std::vector<EdgeId>& result) const;
public:
    explicit ContractionHierarchy(const Graph& g, unsigned witness_limit=128);
    Route route(Node s, Node t) const;
    size_t shortcut_count() const { return arcs_.size()-g_.edges.size(); }
};
}
