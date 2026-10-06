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

## Milestone 4B verified — deterministic greedy batch matching

Implemented and verified on 2026-10-04, based on commit `d980cc8`; changes are
uncommitted. Milestones 1–4A APIs remain available. No Hungarian, concurrency,
or WAL/recovery implementation was added; Milestone 4 as a whole remains open.

### API and behavior

- Added `RideRequest { id, pickup }`, `Assignment { request_id, driver_id,
  pickup_distance }`, `greedy_batch_brute_force()` and `greedy_batch_spatial()`.
- Requests run in supplied order. Nearest available driver wins; exact distance
  ties use lower driver ID. Winners immediately become unavailable. Unmatched
  results contain `invalid` and infinite pickup distance.
- Driver IDs need not equal vector indices. Both APIs validate unique,
  non-sentinel driver/request IDs and finite coordinates before mutation.
- Spatial matching builds one grid per call and uses an ID-to-index lookup.
  Zero cell size selects automatic sizing; negative/nonfinite sizes are rejected.
  Unsupported grid dimensions/sizing are rejected before mutation. Grid numeric
  conversions were bounded to handle far finite queries without integer overflow.
- Pickup cost is projected Euclidean distance, not routed pickup distance.
  Greedy assignment is order-dependent and does not optimize global batch cost.

### Correctness verification

- Added empty/unavailable/depleted cases, explicit expected assignments,
  competing requests, exact ties, coincident drivers, shuffled noncontiguous IDs,
  request ordering, successive batches, multiple cell sizes, boundary/outside
  queries, invalid IDs/coordinates/cell sizes, and pre-mutation rejection checks.
- Randomized equivalence: 60 seeds, each with three cell sizes and 120 requests;
  additional 100,000-driver mixed-availability case with 100 requests.
- Compared assignment IDs and distances exactly, checked final availability,
  and independently replayed greedy choices and checked no duplicate assignment.
- Fixed optional Estonia test handling: missing data is explicitly skipped;
  parser failures and correctness failures now propagate instead of being caught.
  CTest runs in the build directory and skips this optional fixture; the direct
  root-directory run below exercised the actual Estonia graph.
- Final Release CTest: 3/3 passed (`correctness`, `routing`, `matching`), 1.67 s.
  Direct matching tests and Python OSM profile/real parser fixture tests passed.
  `git diff --check` passed. Sanitizers were not run for 4B; no sanitizer timing
  is included in these optimized measurements.

Exact commands, run from the repository root:

```sh
.venv/bin/cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DSANITIZER=
.venv/bin/cmake --build build -j 4
.venv/bin/ctest --test-dir build --output-on-failure
.venv/bin/python tests/test_import.py
./build/matching_tests
./build/batch_matching_bench data/estonia.srg > results/batch-matching-benchmark.txt
git diff --check
```

### Batch benchmark

- New executable/target: `batch_matching_bench`; source:
  `benchmarks/batch_matching_bench.cpp`. Full output:
  `results/batch-matching-benchmark.txt`.
- Release, C++17, Apple LLVM 17.0.0 (clang-1700.6.3.2), macOS 26.5.2
  (25F84), arm64, Mac15,12, 8 logical CPUs, 8 GiB RAM. Hardware was read using
  `uname -m`, `sw_vers`, and `/usr/sbin/sysctl -n hw.model hw.logicalcpu hw.memsize`.
- Estonia graph: 1,328,518 nodes, 2,671,028 directed edges. Simulated drivers and
  requests are sampled from OSM nodes, seeds driver=42 and request=1234567.
  Attribution and data reproduction remain in `docs/OSM.md`; graph/PBF hashes
  remain in `data/estonia.json`. Benchmark fails if the map cannot be loaded.
- 2,000 requests per row; driver counts 1,000/10,000/100,000; reset batch sizes
  1/10/100/1,000 plus a depletion workload carrying reservations across two
  1,000-request batches. All 30,000 assignment comparisons passed, including
  exact distance equality and final availability.
- Timings include validation, lookup/grid construction, matching, result
  allocation and local setup teardown. Input resets, request slicing, and
  correctness comparisons are outside timing. Warm-up uses separate state;
  timed execution order alternates. Standalone grid build is separately reported.

Selected results for reset batches of 1,000 (2 measured batches per scale):

| Drivers | Brute requests/s | Spatial requests/s | Speedup | Mean pickup m (both) |
|---:|---:|---:|---:|---:|
| 1,000 | 637856.598 | 2876836.860 | 4.510x | 10373.680 |
| 10,000 | 48866.849 | 1557026.080 | 31.863x | 772.282 |
| 100,000 | 5146.256 | 226240.070 | 43.962x | 152.588 |

At 100,000 drivers, batch sizes 1/10/100 achieved 0.572x/1.010x/5.367x;
per-call validation and grid construction dominate small batches. Depletion at
100,000 drivers achieved 44.824x, with mean pickup 153.670 m. At 1,000 drivers,
the depletion workload matched 1,000 and explicitly left 1,000 unmatched.
Throughput counts all processed requests, including unmatched ones.

Limitations: this is single-threaded, positions remain fixed, and callers reset
availability explicitly to simulate completion. IDs are unique per input batch;
there is no cross-call request deduplication. Dense grids can consume substantial
memory for tiny explicit cell sizes; unsupported dimensions/allocation failures
occur during setup before reservations. Large-batch p95/p99 values have only two
samples per row and are descriptive, not reliable tail estimates. These full-API
batch results are distinct from the 4A nearest-query benchmark (409.4x); its
benchmark source and saved results were preserved. No 4B requirement is blocked.

## Benchmark suite

`scripts/run_benchmarks.sh` builds Release, runs CTest, and writes a new set of
results under `results/suite/`. Reproduction steps are in `BENCHMARKS.md`.
Earlier files in `results/` are not replaced by that script. The suite does not
add a million-request workload. Dispatch timing is synthetic coordinates, not
routed trips. WAL append timing is one record plus its own `fsync`.
