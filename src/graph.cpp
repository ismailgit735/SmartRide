#include "smartride/graph.hpp"
#include <cmath>
#include <fstream>
#include <stdexcept>
namespace sr {
double distance(Point a, Point b) { return std::hypot(a.x-b.x, a.y-b.y); }
Node Graph::add_node(Point p) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || points.size() >= invalid)
        throw std::invalid_argument("invalid node");
    points.push_back(p); out.emplace_back(); in.emplace_back(); return points.size()-1;
}
EdgeId Graph::add_edge(Node u, Node v, double w) {
    if(u>=points.size() || v>=points.size() || !std::isfinite(w) || w<0 || edges.size()>=invalid)
        throw std::invalid_argument("invalid edge");
    EdgeId id=edges.size(); edges.push_back({u,v,w}); out[u].push_back(id); in[v].push_back(id); return id;
}
void Graph::save(const std::string& path) const {
    std::ofstream f(path); f.precision(17);
    f<<"SRG1 "<<points.size()<<' '<<edges.size()<<'\n';
    for(auto p:points) f<<p.x<<' '<<p.y<<'\n';
    for(auto e:edges) f<<e.from<<' '<<e.to<<' '<<e.weight<<'\n';
    if(!f) throw std::runtime_error("graph write failed");
}
Graph Graph::load(const std::string& path) {
    std::ifstream f(path); std::string magic; uint64_t n,m;
    if(!(f>>magic>>n>>m) || magic!="SRG1" || n>=invalid || m>=invalid)
        throw std::runtime_error("invalid graph header");
    Graph g; for(uint64_t i=0;i<n;++i) { Point p; if(!(f>>p.x>>p.y)) throw std::runtime_error("truncated nodes"); g.add_node(p); }
    for(uint64_t i=0;i<m;++i) { Node u,v; double w; if(!(f>>u>>v>>w)) throw std::runtime_error("truncated edges"); g.add_edge(u,v,w); }
    std::string extra; if(f>>extra) throw std::runtime_error("trailing graph data"); return g;
}
Graph Graph::grid(unsigned side) {
    if(side==0 || uint64_t(side)*side>=invalid) throw std::invalid_argument("grid size");
    Graph g; for(unsigned y=0;y<side;++y) for(unsigned x=0;x<side;++x) g.add_node({100.0*x,100.0*y});
    for(unsigned y=0;y<side;++y) for(unsigned x=0;x<side;++x) {
        Node u=y*side+x;
        if(x+1<side) { g.add_edge(u,u+1,100); g.add_edge(u+1,u,100); }
        if(y+1<side) { g.add_edge(u,u+side,100); g.add_edge(u+side,u,100); }
    } return g;
}
}
