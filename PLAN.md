# SmartRide requirements and completion checklist

- [x] 1. Baseline: CMake/C++17; directed weighted graph; Dijkstra and route
  reconstruction; seeded simulated drivers/requests; nearest dispatch; CLI;
  correctness tests. Verify before OSM work.
- [x] 2. OSM: established parser, real drivable network targeting ~1M directed
  edges, actual counts, one-way handling, access/weight/turn limitations,
  attribution and reproducible public download instructions.
- [x] 3. Routing: A*, bidirectional A*, Contraction Hierarchies; Dijkstra
  distance and reconstructed-route equivalence on directed, disconnected,
  randomized graphs. Record preprocessing cost.
- [ ] 4. Matching: grid, nearest, greedy batch, exact bounded Hungarian;
  100K active drivers; candidate/approximation tradeoffs; identical seeds and
  workload comparison; spatial speedup and pickup cost.
  - [x] 4A: SpatialGrid nearest-driver indexing and oracle benchmark.
  - [x] 4B: deterministic greedy batch matching; validated inputs, immediate
    reservations, brute-force oracle equivalence, 100K-driver tests and benchmark.
  - [ ] Remaining: exact bounded Hungarian and candidate/approximation tradeoffs.
- [ ] 5. Concurrency: std::thread pool, sharded locks, atomic driver state;
  competing requests and double-booking tests; TSan where supported.
- [ ] 6. Recovery: WAL, durable semantics, abrupt death/replay tests,
  duplicates and incomplete-tail tests.
- [ ] 7. Reproducible optimized benchmarks: route/dispatch p50/p95/p99,
  throughput, memory and preprocessing, real-map vs synthetic labels;
  million-request stress if machine supports it; raw results/seeds/hardware/
  exact commands; sanitizer results separately recorded.
- [ ] 8. Delivery: complete source, automated build/test/benchmark scripts,
  README, architecture, tables/plots, technical report, measured resume bullets.

Design decisions: local Python virtual environment for tooling and established
pyosmium/libosmium OSM preprocessing; all graph/routing/matching/concurrency/
recovery runtime code in C++17. Immutable road graph and spatial driver positions;
explicit trip completion returns a driver to availability. Distance weights in
meters with admissible straight-line routing heuristics. Explain limitations.
