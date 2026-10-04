# Progress

## Environment inspection
- macOS 26.5.2, arm64, Mac15,12, 8 logical CPUs, 8 GiB RAM.
- Apple Clang 17.0.0, Git 2.50.1; CMake and Homebrew absent.
- 26 GiB disk available initially. Python 3.12 available.
- Hardware sysctl required sandbox permission, granted.
- Installing workspace-local CMake, pyosmium, plotting tools in `.venv`.
- Repository initially empty. Milestone 1 underway.

## Milestone 1 verified
Dependencies installed locally: CMake 4.4.3, osmium 4.3.1, matplotlib 3.11.2.
Release build and CTest passed; CLI: 900 nodes, 3480 directed edges,
5800 m route, 58 reconstructed edges. Commands:
`.venv/bin/cmake -S . -B build -DCMAKE_BUILD_TYPE=Release`,
`.venv/bin/cmake --build build -j 4`, `.venv/bin/ctest --test-dir build --output-on-failure`,
`./build/smartride_cli`. Beginning real OSM import.

OSM importer profile tests and a real pyosmium XML fixture passed. Geofabrik
PBF download is currently stalled before receiving data; diagnosing connectivity.

Download diagnosis: IPv4 works. Overlapping initial and IPv4 retries corrupted
the same destination; parser rejected the result. Stopped the old transfer and
started a clean download to a distinct temporary path. Reproduction script now
uses temporary download plus rename and bounded connect/total timeouts.

## Milestone 2 verified
Real Estonia graph imported: 1,328,518 nodes and 2,671,028 directed edges,
181,565 accepted ways; 1,927 unsupported turn restriction relations recorded.
Import took 29.3897 s. Actual network exceeds the approximate 1M-edge target;
retained the whole extract instead of arbitrarily deleting roads. Metadata and
SHA256 hashes are in data/estonia.json and results/import.json. C++ load/demo
succeeded: 185,094 m route with 2,937 original edges. Profile/parser fixture tests
pass. ASan startup stalled both in/outside sandbox; interrupted and marked
unverified pending sanitizer troubleshooting. Starting routing algorithms.

## Milestone 3 verified
Contraction Hierarchies shortcut generation fixed in `src/routing.cpp`:
- Candidate incoming/outgoing edges deduplicated to select minimum weights per neighbor.
- Direct edges inspected during witness evaluation so direct alternatives are recognized even when `witness_limit == 0`.
- Release build passes 100% of test suites via CTest in ~0.10 s:
  - Test #1: `correctness` passed (Dijkstra, graph validation, nearest driver).
  - Test #2: `routing` passed (A*, balanced bidirectional A*, Contraction Hierarchies against Dijkstra oracle on directed, disconnected, cycle, parallel, grid, and 60 randomized graph seeds).
- Python OSM profile and pyosmium parser tests passed in `tests/test_import.py`.
- Results captured in `results/routing-tests.txt`. Next milestone: Milestone 4 (Spatial Indexing & Matching).
## Milestone 4A verified — SpatialGrid benchmark

SpatialGrid nearest-driver correctness and performance benchmark completed.

- Compared `sr::nearest_driver()` brute-force lookup against `sr::SpatialGrid::nearest_driver()`.
- Benchmarked 1,000, 10,000, and 100,000 active drivers.
- Used 2,000 reproducible queries per scale.
- Fixed random seeds: driver=42, query=1234567.
- Benchmark ran on the Estonia road network: 1,328,518 nodes and 2,671,028 directed edges.
- Verified every SpatialGrid result matched the brute-force result.
- All three benchmark scales achieved 100% correctness.
- Recorded SpatialGrid construction time, p50/p95/p99 latency, QPS, and QPS speedup.
- Raw results saved to `results/matching-benchmark.txt`.
- Reproducible benchmark target: `matching_bench`.
- Release build, CTest, and Python import/parser tests passed.
- Batch matching and Hungarian matching were intentionally not implemented.

### Benchmark results

| Drivers | Grid Build (ms) | Brute p50 (us) | Brute p95 (us) | Brute p99 (us) | Brute QPS | Grid p50 (us) | Grid p95 (us) | Grid p99 (us) | Grid QPS | QPS Speedup |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 1,000 | 0.02 | 1.96 | 2.33 | 2.54 | 491778 | 0.17 | 0.38 | 0.58 | 5060614 | 10.3x |
| 10,000 | 0.19 | 19.12 | 22.58 | 24.08 | 50443 | 0.17 | 0.58 | 1.62 | 3884438 | 77.0x |
| 100,000 | 1.89 | 192.88 | 207.92 | 244.50 | 5127 | 0.29 | 1.33 | 3.08 | 2099002 | 409.4x |
