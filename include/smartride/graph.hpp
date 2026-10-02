#pragma once
#include <cstdint>
#include <limits>
#include <string>
#include <vector>
namespace sr {
using Node = uint32_t;
using EdgeId = uint32_t;
constexpr Node invalid = std::numeric_limits<uint32_t>::max();
constexpr double inf = std::numeric_limits<double>::infinity();
struct Point { double x = 0, y = 0; };
double distance(Point a, Point b);
struct Edge { Node from, to; double weight; };
class Graph {
public:
    std::vector<Point> points;
    std::vector<Edge> edges;
    std::vector<std::vector<EdgeId>> out, in;
    Node add_node(Point p = {});
    EdgeId add_edge(Node u, Node v, double w);
    void save(const std::string& path) const;
    static Graph load(const std::string& path);
    static Graph grid(unsigned side);
};
}
