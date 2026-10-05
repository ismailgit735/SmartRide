#include "smartride/routing.hpp"
#include "smartride/matching.hpp"
#include <cmath>
#include <cstdio>
#include <iostream>
#include <fstream>
#include <limits>
#include <stdexcept>
#define CHECK(x) do { if(!(x)) throw std::runtime_error("check failed: " #x); } while(false)
void malformed_graph_tests() {
    const char* path="test_malformed.srg";
    struct Cleanup { const char* path; ~Cleanup() { std::remove(path); } } cleanup{path};
    for (const char* contents : {
        "", "BAD 0 0", "SRG1", "SRG1 -1 0", "SRG1 0 -1",
        "SRG1 4294967295 0", "SRG1 0 4294967295",
        "SRG1 18446744073709551616 0", "SRG1 1 0\n0",
        "SRG1 1 0\nnan 0", "SRG1 1 0\n0 inf", "SRG1 1 0\n1e309 0",
        "SRG1 1 1\n0 0\n", "SRG1 1 1\n0 0\n0 0",
        "SRG1 1 1\n0 0\n1 0 1", "SRG1 1 1\n0 0\n0 1 1",
        "SRG1 1 1\n0 0\n-1 0 1", "SRG1 1 1\n0 0\n4294967296 0 1",
        "SRG1 1 1\n0 0\n0 0 -1", "SRG1 1 1\n0 0\n0 0 nan",
        "SRG1 1 1\n0 0\n0 0 inf", "SRG1 1 1\n0 0\n0 0 1e309",
        "SRG1 0 0\nextra", "SRG1 0 0\n0 0"
    }) {
        { std::ofstream f(path); f << contents; CHECK(bool(f)); }
        bool rejected=false;
        try { (void)sr::Graph::load(path); }
        catch (const std::runtime_error&) { rejected=true; }
        catch (const std::invalid_argument&) { rejected=true; }
        CHECK(rejected);
    }
    { std::ofstream f(path); f << "SRG1 1 1\n0 0\n0 0 0\n  \n"; }
    auto g=sr::Graph::load(path);
    CHECK(g.points.size()==1 && g.edges.size()==1 && g.edges[0].weight==0);
    std::remove(path);
    bool rejected=false;
    try { (void)sr::Graph::load(path); } catch (const std::runtime_error&) { rejected=true; }
    CHECK(rejected);
}
int main() {
    try {
        malformed_graph_tests();
        sr::Graph g; for(int i=0;i<4;++i) g.add_node();
        for (double bad : {sr::inf, -sr::inf, std::numeric_limits<double>::quiet_NaN()}) {
            for (sr::Point p : {sr::Point{bad,0}, sr::Point{0,bad}}) {
                bool rejected=false;
                try { g.add_node(p); } catch (const std::invalid_argument&) { rejected=true; }
                CHECK(rejected && g.points.size()==4 && g.out.size()==4 && g.in.size()==4);
            }
            bool rejected=false;
            try { g.add_edge(0,1,bad); } catch (const std::invalid_argument&) { rejected=true; }
            CHECK(rejected && g.edges.empty());
        }
        for (auto endpoints : {std::pair<sr::Node,sr::Node>{4,0}, {0,4}}) {
            bool rejected=false;
            try { g.add_edge(endpoints.first,endpoints.second,1); }
            catch (const std::invalid_argument&) { rejected=true; }
            CHECK(rejected && g.edges.empty());
        }
        g.add_edge(0,1,2); g.add_edge(1,2,3); g.add_edge(0,2,8);
        auto r=sr::dijkstra(g,0,2); CHECK(r.distance==5); CHECK(sr::valid_route(g,0,2,r));
        CHECK(!std::isfinite(sr::dijkstra(g,2,0).distance)); CHECK(!std::isfinite(sr::dijkstra(g,0,3).distance));
        CHECK(sr::dijkstra(g,0,0).distance==0);
        for (double bad : {std::numeric_limits<double>::quiet_NaN(), -sr::inf, -1.0})
            CHECK(!sr::valid_route(g,0,0,sr::Route{bad,{}}));
        CHECK(!sr::valid_route(g,0,0,sr::Route{sr::inf,{}}));
        CHECK(!sr::valid_route(g,99,99,sr::Route{0,{}}));
        CHECK(!sr::valid_route(g,0,99,sr::Route{sr::inf,{}}));
        CHECK(!sr::valid_route(g,0,2,sr::Route{5,{sr::invalid}}));
        CHECK(!sr::valid_route(g,0,2,sr::Route{5,{1,0}}));
        CHECK(!sr::valid_route(g,0,2,sr::Route{4,{0,1}}));
        CHECK(!sr::valid_route(g,0,2,sr::Route{sr::inf,{0,1}}));
        sr::Graph huge; huge.add_node(); huge.add_node(); huge.add_node();
        huge.add_edge(0,1,1e308); huge.add_edge(1,2,1e308);
        CHECK(!sr::valid_route(huge,0,2,sr::Route{1e308,{0,1}}));
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
