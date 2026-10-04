#include "smartride/routing.hpp"
#include "smartride/matching.hpp"
#include <cmath>
#include <cstdio>
#include <iostream>
#include <stdexcept>
#define CHECK(x) do { if(!(x)) throw std::runtime_error("check failed: " #x); } while(false)
int main() {
    try {
        sr::Graph g; for(int i=0;i<4;++i) g.add_node();
        g.add_edge(0,1,2); g.add_edge(1,2,3); g.add_edge(0,2,8);
        auto r=sr::dijkstra(g,0,2); CHECK(r.distance==5); CHECK(sr::valid_route(g,0,2,r));
        CHECK(!std::isfinite(sr::dijkstra(g,2,0).distance)); CHECK(!std::isfinite(sr::dijkstra(g,0,3).distance));
        CHECK(sr::dijkstra(g,0,0).distance==0);
        bool rejected=false;
        try { g.add_edge(0,1,-1); } catch(const std::invalid_argument&) { rejected=true; }
        CHECK(rejected);
        g.save("test_roundtrip.srg"); auto copy=sr::Graph::load("test_roundtrip.srg");
        std::remove("test_roundtrip.srg");
        CHECK(copy.edges.size()==g.edges.size()); CHECK(sr::dijkstra(copy,0,2).distance==5);
        auto grid=sr::Graph::grid(10); CHECK(sr::dijkstra(grid,0,99).distance==1800);
        auto drivers=sr::simulate_drivers(1000,grid,42); CHECK(drivers.size()==1000);
        std::vector<sr::Driver> d={{0,{0,0},true},{1,{10,0},true}};
        CHECK(sr::nearest_driver(d,{9,0})==1); d[1].available=false; CHECK(sr::nearest_driver(d,{9,0})==0);
        d[0].available=false; CHECK(sr::nearest_driver(d,{0,0})==sr::invalid);
        d[0].available=true; d[1].available=true;
        sr::SpatialGrid sgrid(d, 5.0);
        CHECK(sgrid.nearest_driver({9,0})==1); d[1].available=false; CHECK(sgrid.nearest_driver({9,0})==0);
        d[0].available=false; CHECK(sgrid.nearest_driver({0,0})==sr::invalid);
        std::cout<<"baseline tests passed\n";
    } catch(const std::exception& e) {std::cerr<<e.what()<<'\n'; return 1;}
}
