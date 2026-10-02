#include "smartride/routing.hpp"
#include <cmath>
#include <iostream>
#include <random>
#include <stdexcept>
void compare(const sr::Graph& g,unsigned limit=128) {
    sr::Router router(g); sr::ContractionHierarchy ch(g,limit);
    for(sr::Node s=0;s<g.points.size();++s) for(sr::Node t=0;t<g.points.size();++t) {
        auto ref=sr::dijkstra(g,s,t);
        for(auto r:{router.astar(s,t),router.bidirectional_astar(s,t),ch.route(s,t)}) {
            bool same=(!std::isfinite(ref.distance) && !std::isfinite(r.distance)) || std::abs(ref.distance-r.distance)<1e-7*std::max(1.0,ref.distance);
            if(!same || !sr::valid_route(g,s,t,r)) throw std::runtime_error("routing mismatch s="+std::to_string(s)+" t="+std::to_string(t));
        }
    }
}
int main() {
    try {
        sr::Graph directed; for(int i=0;i<7;++i) directed.add_node({double(i),0});
        directed.add_edge(0,1,0); directed.add_edge(1,2,0); directed.add_edge(2,0,0);
        directed.add_edge(2,3,4); directed.add_edge(0,3,9); directed.add_edge(0,3,5);
        directed.add_edge(3,4,2); directed.add_edge(4,4,0); directed.add_edge(6,5,1);
        compare(directed); compare(directed,0); compare(sr::Graph::grid(8));
        std::mt19937_64 rng(918273);
        for(int test=0;test<60;++test) {
            sr::Graph g; unsigned n=5+rng()%30;
            for(unsigned i=0;i<n;++i) g.add_node({double(rng()%100),double(rng()%100)});
            for(unsigned i=0;i<n*4;++i) {sr::Node u=rng()%n,v=rng()%n;
                double w=test%2?sr::distance(g.points[u],g.points[v])+rng()%20:double(rng()%30);
                g.add_edge(u,v,w);
            }
            compare(g,test%3?128:0);
        }
        std::cout<<"A*, balanced bidirectional A*, CH: all-pairs directed/disconnected/zero/parallel/random route checks passed; seed=918273\n";
    } catch(const std::exception& e) {std::cerr<<e.what()<<'\n'; return 1;}
}
