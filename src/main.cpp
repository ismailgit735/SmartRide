#include "smartride/routing.hpp"
#include "smartride/matching.hpp"
#include <iostream>
int main(int argc,char** argv) {
    try {
        auto g=argc>1 ? sr::Graph::load(argv[1]) : sr::Graph::grid(30);
        auto drivers=sr::simulate_drivers(1000,g,42);
        auto r=sr::dijkstra(g,0,g.points.size()-1);
        std::cout<<"nodes="<<g.points.size()<<" directed_edges="<<g.edges.size()
                 <<" route_m="<<r.distance<<" route_edges="<<r.edges.size()
                 <<" nearest_driver="<<sr::nearest_driver(drivers,g.points.back())<<'\n';
    } catch(const std::exception& e) { std::cerr<<e.what()<<'\n'; return 1; }
}
