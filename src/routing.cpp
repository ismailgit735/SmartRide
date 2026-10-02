#include "smartride/routing.hpp"
#include <algorithm>
#include <cmath>
#include <queue>
#include <stdexcept>
namespace sr {
Route dijkstra(const Graph& g, Node s, Node t) {
    if(s>=g.points.size() || t>=g.points.size()) throw std::out_of_range("route endpoint");
    std::vector<double> d(g.points.size(),inf); std::vector<EdgeId> prev(d.size(),invalid);
    using Entry=std::pair<double,Node>; std::priority_queue<Entry,std::vector<Entry>,std::greater<Entry>> q;
    d[s]=0; q.push({0,s}); Route r;
    while(!q.empty()) {
        auto [cost,u]=q.top(); q.pop(); if(cost!=d[u]) continue; ++r.settled;
        if(u==t) break;
        for(auto id:g.out[u]) { auto e=g.edges[id]; if(cost+e.weight<d[e.to]) { d[e.to]=cost+e.weight; prev[e.to]=id; q.push({d[e.to],e.to}); } }
    }
    r.distance=d[t]; if(!std::isfinite(d[t])) return r;
    for(Node u=t;u!=s;) { auto id=prev[u]; r.edges.push_back(id); u=g.edges[id].from; }
    std::reverse(r.edges.begin(),r.edges.end()); return r;
}
bool valid_route(const Graph& g, Node s, Node t, const Route& r) {
    if(!std::isfinite(r.distance)) return r.edges.empty();
    double sum=0; Node u=s;
    for(auto id:r.edges) { if(id>=g.edges.size()) return false; auto e=g.edges[id]; if(e.from!=u) return false; u=e.to; sum+=e.weight; }
    return u==t && std::abs(sum-r.distance)<=1e-7*std::max(1.0,sum);
}
}

namespace sr {
namespace {
using Entry=std::pair<double,Node>;
using Queue=std::priority_queue<Entry,std::vector<Entry>,std::greater<Entry>>;
void endpoints(const Graph& g,Node s,Node t) {
    if(s>=g.points.size() || t>=g.points.size()) throw std::out_of_range("route endpoint");
}
void join_path(const Graph& g,Node s,Node t,Node meet,const std::vector<EdgeId>& pf,
               const std::vector<EdgeId>& pb,Route& r) {
    for(Node v=meet;v!=s;) { auto id=pf[v]; r.edges.push_back(id); v=g.edges[id].from; }
    std::reverse(r.edges.begin(),r.edges.end());
    for(Node v=meet;v!=t;) { auto id=pb[v]; r.edges.push_back(id); v=g.edges[id].to; }
}
}
Router::Router(const Graph& g):g_(g) {
    for(auto e:g.edges) { double len=distance(g.points[e.from],g.points[e.to]); if(len>0) scale_=std::min(scale_,e.weight/len); }
    scale_*=1-1e-12; // Conservative margin for floating point reduced costs.
}
Route Router::astar(Node s,Node t) const {
    endpoints(g_,s,t); auto h=[&](Node v){return scale_*distance(g_.points[v],g_.points[t]);};
    std::vector<double> d(g_.points.size(),inf); std::vector<EdgeId> prev(d.size(),invalid);
    Queue q; d[s]=0; q.push({h(s),s}); Route r;
    while(!q.empty()) {
        auto [key,u]=q.top(); q.pop(); if(key>d[u]+h(u)) continue;
        ++r.settled; if(u==t) break;
        for(auto id:g_.out[u]) { auto e=g_.edges[id]; double nd=d[u]+e.weight;
            if(nd<d[e.to]) { d[e.to]=nd; prev[e.to]=id; q.push({nd+h(e.to),e.to}); }
        }
    }
    r.distance=d[t]; if(!std::isfinite(r.distance)) return r;
    for(Node v=t;v!=s;) { auto id=prev[v]; r.edges.push_back(id); v=g_.edges[id].from; }
    std::reverse(r.edges.begin(),r.edges.end()); return r;
}
Route Router::bidirectional_astar(Node s,Node t) const {
    endpoints(g_,s,t); Route r; if(s==t) {r.distance=0; return r;}
    auto p=[&](Node v){return scale_*(distance(g_.points[v],g_.points[t])-distance(g_.points[s],g_.points[v]))/2;};
    double ps=p(s),pt=p(t);
    const size_t n=g_.points.size(); std::vector<double> df(n,inf),db(n,inf);
    std::vector<EdgeId> pf(n,invalid),pb(n,invalid); Queue f,b;
    df[s]=db[t]=0; f.push({0,s}); b.push({0,t}); Node meet=invalid;
    auto clean=[&](Queue& q,const std::vector<double>& d,bool forward) {
        while(!q.empty()) { auto [key,u]=q.top(); double expected=d[u]+(forward?p(u)-ps:pt-p(u));
            if(key<=expected) break; q.pop(); }
    };
    while(true) {
        clean(f,df,true); clean(b,db,false); if(f.empty() || b.empty()) break;
        if(std::isfinite(r.distance) && f.top().first+b.top().first>=r.distance+pt-ps) break;
        bool forward=f.top().first<=b.top().first; auto& q=forward?f:b;
        Node u=q.top().second; q.pop(); ++r.settled;
        auto& d=forward?df:db; auto& prev=forward?pf:pb;
        if(df[u]+db[u]<r.distance) {r.distance=df[u]+db[u]; meet=u;}
        for(auto id:forward?g_.out[u]:g_.in[u]) {
            auto e=g_.edges[id]; Node v=forward?e.to:e.from; double nd=d[u]+e.weight;
            if(nd<d[v]) {d[v]=nd; prev[v]=id; q.push({nd+(forward?p(v)-ps:pt-p(v)),v});}
            if(df[v]+db[v]<r.distance) {r.distance=df[v]+db[v]; meet=v;}
        }
    }
    if(meet!=invalid) join_path(g_,s,t,meet,pf,pb,r); return r;
}

ContractionHierarchy::ContractionHierarchy(const Graph& g,unsigned witness_limit):g_(g),out_(g.out),in_(g.in),rank_(g.points.size(),invalid) {
    arcs_.reserve(g.edges.size()*2);
    for(EdgeId id=0;id<g.edges.size();++id) {auto e=g.edges[id]; arcs_.push_back({e.from,e.to,e.weight,invalid,invalid,id});}
    std::vector<unsigned> level(g.points.size(),0);
    auto priority=[&](Node v) {
        uint64_t ni=0,no=0;
        for(auto id:in_[v]) if(arcs_[id].from!=v && rank_[arcs_[id].from]==invalid) ++ni;
        for(auto id:out_[v]) if(arcs_[id].to!=v && rank_[arcs_[id].to]==invalid) ++no;
        return double(ni*no+ni+no+8*level[v]);
    };
    Queue order; for(Node v=0;v<g.points.size();++v) order.push({priority(v),v});
    std::vector<double> wd(g.points.size(),inf); std::vector<Node> touched;
    // One bounded witness search per incoming source; all outgoing targets share it.
    auto witness=[&](Node source,Node avoid,double limit) {
        for(auto v:touched) wd[v]=inf; touched.clear();
        Queue q; wd[source]=0; touched.push_back(source); q.push({0,source}); unsigned settled=0;
        while(!q.empty() && settled<witness_limit) {
            auto [cost,u]=q.top(); q.pop(); if(cost!=wd[u]) continue;
            if(cost>limit) break; ++settled;
            for(auto id:out_[u]) { const auto& a=arcs_[id];
                if(a.to==avoid || rank_[a.to]!=invalid) continue;
                double nd=cost+a.weight;
                if(nd<=limit && nd<wd[a.to]) {if(!std::isfinite(wd[a.to])) touched.push_back(a.to); wd[a.to]=nd; q.push({nd,a.to});}
            }
        }
    };
    uint32_t next=0;
    while(!order.empty()) {
        auto [old,v]=order.top(); order.pop(); if(rank_[v]!=invalid) continue;
        double actual=priority(v);
        if(actual>old && !order.empty() && actual>order.top().first) {order.push({actual,v}); continue;}
        std::vector<EdgeId> incoming,outgoing;
        for(auto id:in_[v]) if(arcs_[id].from!=v && rank_[arcs_[id].from]==invalid) incoming.push_back(id);
        for(auto id:out_[v]) if(arcs_[id].to!=v && rank_[arcs_[id].to]==invalid) outgoing.push_back(id);
        for(auto left:incoming) {
            const auto a=arcs_[left]; double limit=0;
            for(auto right:outgoing) limit=std::max(limit,a.weight+arcs_[right].weight);
            witness(a.from,v,limit);
            for(auto right:outgoing) {
                const auto b=arcs_[right]; if(a.from==b.to) continue;
                double w=a.weight+b.weight;
                if(wd[b.to]<=w) continue;
                if(arcs_.size()>=invalid) throw std::overflow_error("too many CH arcs");
                EdgeId id=arcs_.size(); arcs_.push_back({a.from,b.to,w,left,right,invalid});
                out_[a.from].push_back(id); in_[b.to].push_back(id);
            }
        }
        rank_[v]=next++;
        for(auto id:incoming) level[arcs_[id].from]=std::max(level[arcs_[id].from],level[v]+1);
        for(auto id:outgoing) level[arcs_[id].to]=std::max(level[arcs_[id].to],level[v]+1);
    }
}
void ContractionHierarchy::unpack(EdgeId id,std::vector<EdgeId>& result) const {
    std::vector<EdgeId> stack{id};
    while(!stack.empty()) {auto a=arcs_[stack.back()]; stack.pop_back();
        if(a.original!=invalid) result.push_back(a.original);
        else {stack.push_back(a.right); stack.push_back(a.left);}
    }
}
Route ContractionHierarchy::route(Node s,Node t) const {
    endpoints(g_,s,t); Route r; if(s==t) {r.distance=0; return r;}
    const auto n=rank_.size(); std::vector<double> df(n,inf),db(n,inf);
    std::vector<EdgeId> pf(n,invalid),pb(n,invalid); Queue f,b;
    df[s]=db[t]=0; f.push({0,s}); b.push({0,t}); Node meet=invalid;
    // Each frontier independently stops at mu. Sum-of-minima is not a valid
    // bound for CH's two different upward subgraphs.
    while(!f.empty() || !b.empty()) {
        bool forward=b.empty() || (!f.empty() && f.top().first<=b.top().first);
        auto& q=forward?f:b; auto& d=forward?df:db; auto& prev=forward?pf:pb;
        auto [cost,u]=q.top(); q.pop(); if(cost!=d[u] || cost>r.distance) continue; ++r.settled;
        if(df[u]+db[u]<r.distance) {r.distance=df[u]+db[u]; meet=u;}
        for(auto id:forward?out_[u]:in_[u]) {
            const auto& a=arcs_[id]; Node v=forward?a.to:a.from;
            if(rank_[v]<=rank_[u]) continue;
            double nd=cost+a.weight;
            if(nd<d[v]) {d[v]=nd; prev[v]=id; q.push({nd,v});}
            if(df[v]+db[v]<r.distance) {r.distance=df[v]+db[v]; meet=v;}
        }
    }
    if(meet==invalid) return r;
    std::vector<EdgeId> path;
    for(Node v=meet;v!=s;) {auto id=pf[v]; path.push_back(id); v=arcs_[id].from;}
    std::reverse(path.begin(),path.end());
    for(Node v=meet;v!=t;) {auto id=pb[v]; path.push_back(id); v=arcs_[id].to;}
    for(auto id:path) unpack(id,r.edges); return r;
}
}
