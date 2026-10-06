# SmartRide

SmartRide is a C++17 ride-matching and road-routing engine. It loads a directed
OpenStreetMap graph, computes distance-minimizing routes, matches drivers by
straight-line pickup cost, reserves drivers from a thread pool, and can record
those reservations in a write-ahead log.

There is no GUI, no live traffic model, and no claim of production or
cloud-scale deployment. Measured numbers below come from `results/suite/`.
Older files in `results/` are earlier runs and are not the numbers cited here.

## Problem

Given a fixed road network and a set of driver positions, the engine has to
answer three questions without double-booking a driver:

1. What is a shortest path, in meters, between two intersections?
2. Which available driver is closest to a pickup, using projected Euclidean
   distance rather than a routed trip?
3. If the process dies after a successful assignment, which reservations are
   still committed?

Routing cost is haversine segment length. Pickup matching does not route from
the driver to the rider.

## Architecture

Python is used only to turn an OSM PBF into an `SRG1` graph. The C++ runtime
loads that graph, routes, matches, dispatches, and writes the log.

```text
OSM PBF
  |  scripts/import_osm.py (pyosmium)
  v
SRG1 graph (immutable directed edges)
  |
  +-- Dijkstra / A* / bidirectional A* / Contraction Hierarchies
  |
  +-- SpatialGrid, nearest-driver, greedy batch, bounded Hungarian
  |
  +-- Dispatcher (std::thread workers, atomic Available -> Reserved)
        |
        +-- optional WriteAheadLog (one record, then fsync)
```

Driver positions stay fixed for the life of a dispatcher or matching call.
`release()` is the explicit return to availability. Queue locking and the WAL
commit lock are separate from the atomic driver word.

## OpenStreetMap graph

The checked-in measurements use the Geofabrik Estonia extract. Import counts
in `data/estonia.json`:

| | |
|---|---|
| Nodes | 1,328,518 |
| Directed edges | 2,671,028 |
| Accepted ways | 181,565 |
| Import time | 29.3897 s |
| Turn restrictions counted, not applied | 1,927 |

The PBF and `.srg` are local and not stored in Git. Attribution, the highway
profile, one-way rules, and the 59° equirectangular projection are in
[docs/OSM.md](docs/OSM.md). Weights are nonnegative meters. Disconnected roads
are kept; an unreachable query returns infinity and an empty path.

`SRG1` is a text graph: a header, projected coordinates, then
`from, to, weight` records. The loader rejects bad endpoints, negative or
non-finite weights, and truncated input.

## Routing

Dijkstra is the reference. `valid_route` checks that a path's edges sum to its
distance. A\* and bidirectional A\* live on `Router`. Contraction Hierarchies
live on `ContractionHierarchy` with a default witness limit of 128.

A\* uses a consistent scaled Euclidean lower bound. Bidirectional A\* searches
forward on outgoing edges and backward on incoming edges, with the balanced
potentials described in [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md).
Contraction adds a shortcut only when a bounded witness search cannot certify
an equal or shorter alternative. Queries follow upward ranks and unpack
shortcuts back to original edge ids.

On the suite's 100 seeded pairs, all three faster algorithms matched Dijkstra
before any timing sample was recorded. Agreement is
`abs(delta) < 1e-7 * max(1, reference)`, or both distances non-finite.

## Driver matching

Pickup cost is projected Euclidean meters. These APIs are single-threaded.

`nearest_driver` scans every available driver. `SpatialGrid` indexes the same
positions and must return the same driver. Exact distance ties keep the lower
driver id.

`greedy_batch_brute_force` and `greedy_batch_spatial` walk requests in input
order and reserve the nearest available driver immediately. The result depends
on that order. It is not a minimum-cost assignment of the whole batch.

`hungarian_batch` is exact for one batch: maximum cardinality, then minimum
total Euclidean pickup. It does not fall back to greedy. The hard caps are 64
requests and 512 available drivers. Larger inputs throw before any driver
state changes.

The 100,000-driver figures in the suite are for this single-threaded matching
code. They are not dispatcher capacity.

## Concurrent dispatcher

`Dispatcher` keeps one worker pool (`std::thread`). A request id is unique for
that object; a repeat is `Duplicate` and is not matched. Matching calls the
existing `nearest_driver` on a private snapshot, then tries to reserve.

The reservation linearization point is a successful
`compare_exchange_strong` from `Available` to `Reserved`. A failed exchange
means another request took that driver, and the worker retries. No other
transition is valid. `Reserved` returns to `Available` only through
`release()`.

With a WAL path, the commit mutex is held from that successful exchange until
`fsync` returns. `Assigned` is reported only after that `fsync`. A failure
before any record bytes are written rolls the reservation back. If the bytes
were written and `fsync` then fails, the driver stays `Reserved` and the
caller does not receive `Assigned`.

`tests/dispatch_test.cpp` checks single-driver stampedes (1 driver, 8 workers,
200 requests), reuse under contention (4 drivers, 8 workers, 4 submitters, 12
cycles of 48 requests, seed 918273), and a 64-driver / 4,000-request drain
with 8 workers. Those tests require unique winners and no lost request ids.
They are deterministic functional tests, not a million-request benchmark.

## Write-ahead log

A committed assignment or release is one 20-byte little-endian record:

| Field | Size | Value |
|---|---|---|
| magic | u32 | `SRW1` |
| version | u8 | 1 |
| type | u8 | 1 assignment, 2 release |
| length | u16 | 8 |
| request id | u32 | ride id, or `invalid` on release |
| driver id | u32 | |
| crc32 | u32 | IEEE CRC of the first 16 bytes |

Durability is a successful POSIX `fsync` on the log file after the full
record is written. `fflush` is not used and is not treated as durable. The
parent directory is `fsync`ed once when the log is opened. Each record is
`fsync`ed individually. Pickup distance is not stored.

Replay reads complete frames in order. Applying the same assignment twice is
a no-op. A second live owner for one driver, or one request committed to two
drivers, throws. A durable release clears that driver. Reopening a
`Dispatcher` on the same path applies the log before workers start.

If the file size is not a multiple of 20, the short tail is discarded with
`ftruncate` and is not applied. A full 20-byte frame with a bad magic,
version, type, length, or CRC throws. That file is not partially applied, and
the bad frame is left in place.

`tests/wal_test.cpp` forks a child and `SIGKILL`s it at four boundaries:
before any WAL byte is written, after the record is written but before
`fsync`, after `fsync` but before `Assigned` is returned, and after a release
record is durable but before the in-memory transition to `Available`.
Recovery follows the bytes that remain. A power loss between `write` and
`fsync` can still drop a record; killing the process does not reproduce that
power loss.

## Testing

`scripts/build_test.sh` configures Release when no sanitizer is set, builds,
runs CTest, and runs `tests/test_import.py`. CTest names are `correctness`,
`routing`, `matching`, `dispatch`, and `wal`.
`results/suite/index.txt` records that this CTest run passed before the
benchmarks were written. It does not store per-test durations.

Routing tests compare A\*, bidirectional A\*, and Contraction Hierarchies with
Dijkstra on small directed graphs, including disconnected and randomized
cases. Matching tests include a 100,000-driver greedy case. That case is
single-threaded matching, not the dispatcher.

ThreadSanitizer was attempted on this Apple arm64 machine. The Apple TSan
runtime crashed before `main()`, including on the single-threaded correctness
test. Those runs are not evidence that the dispatcher is free of data races.
`PROGRESS.md` also records an AddressSanitizer startup failure that was left
unverified. Sanitizer builds are not mixed into the Release numbers in
`results/suite/`.

## Benchmarks

Cite `results/suite/`. Files such as `results/routing-benchmark.txt` and
`results/matching-benchmark.txt` are older measurements and were not replaced
by the suite. The suite files record commit `e3c4cc9` and a dirty worktree:
the harness that produced them was not yet committed. Do not treat the later
suite commit as the measured revision.

The run is Release, Apple LLVM 17.0.0, C++17, `-O3 -DNDEBUG`, Darwin 25.5.0
arm64, timestamp 2026-10-06T06:24:47Z. The map is `data/estonia.srg`.
Percentile index is `floor(p * n)`. A failed correctness check aborts before
that result file is written. This suite does not contain a million-request
run.

Routing and Contraction Hierarchies use 100 distinct node pairs from
`mt19937_64` seed 918273. Correctness against Dijkstra runs first. Latency is
the query call only. The first 10 pairs are warmup and are excluded. Router
construction, 12.041 ms, is excluded from the routing table. Of 100 pairs, 88
were reachable.

| Algorithm | p50 ms | p95 ms | p99 ms | Mean ms | Queries/sec |
|---|---:|---:|---:|---:|---:|
| Dijkstra | 43.899 | 91.128 | 95.943 | 44.927 | 22.259 |
| A\* | 11.213 | 68.593 | 118.601 | 20.090 | 49.776 |
| Bidirectional A\* | 20.486 | 107.694 | 153.615 | 29.891 | 33.455 |
| Contraction hierarchy | 0.599 | 0.706 | 0.876 | 0.597 | 1675.512 |

Contraction preprocessing took 2406.885 ms and added 2,312,617 shortcuts.
Process RSS was 338,427,904 bytes before preprocessing, 543,719,424 after, and
545,423,360 at peak (`getrusage` on Darwin, in bytes). Those are process RSS
figures, not the size of the hierarchy object. The type does not expose its
own byte count.

Nearest-driver and greedy-batch workloads use driver seed 42 and request seed
1234567, with drivers and requests sampled from OSM nodes. Nearest-driver
timing is the query call. At 100,000 drivers and 2,000 queries, every grid
answer matched brute force. Brute-force p50 was 233.08 µs (3,720 queries/sec).
SpatialGrid p50 was 0.46 µs (1,266,123 queries/sec).

Greedy batch at 100,000 drivers, reset batches of 1,000 requests, processed
2,000/2,000 checked assignments: brute force 4,175.076 requests/sec, spatial
189,668.582 requests/sec. That row has two samples. Its p95 and p99 are
descriptive, not stable tail estimates. Smaller batches are in
`results/suite/batch-matching-benchmark.txt`.

Hungarian batches restore availability each time and compare cardinality and
total pickup with greedy outside the timer. The measured range is 8–64
requests and 32–512 drivers. At 32 requests and 256 drivers, greedy
p50/p95/p99 was 0.029/0.035/0.041 ms and Hungarian was 0.038/0.042/0.044 ms,
with 2,000/2,000 checks. The full table is
`results/suite/hungarian-matching-benchmark.txt`.

Dispatcher benchmarks use synthetic coordinates, not the road graph and not a
WAL. Serial latency is one `submit()` through `drain()` on an idle one-worker
dispatcher: queue wait, nearest-driver matching, and result publication. It
excludes `release()` and excludes `fsync`. Across 64 timed samples, p50 was
0.008 ms, p95 0.009 ms, p99 0.018 ms, and the mean 0.008 ms.

Throughput is the same submit-to-drain boundary for 8 rounds of 64 requests
on 64 drivers. `release()` is after the timer. All 512 requests were assigned
at 1, 4, and 8 workers (918,524.047, 879,790.088, and 926,209.001 assignments/sec). A
separate contention wave used 8 workers, 4 drivers, and 64 requests at one
pickup: 4 assigned, 60 unmatched, 0.093 ms, and no duplicate driver. These
figures are not 100,000-driver dispatcher results.

WAL append latency is one `append_assignment`: the 20-byte write plus that
record's `fsync`. Forty-eight timed records took 2.254 ms total
(21,298.234 records/sec); p50/p95/p99/mean were 0.043/0.063/0.180/0.047 ms.
The dispatcher WAL sample is 16 one-at-a-time assignments. Its latency is
`submit()` through `drain()`, so it includes queue wait, matching, and the
per-record `fsync`. p50/p95/p99/mean were 0.078/0.211/0.211/0.089 ms.
Replay was checked before the file was written.

## Reproduction

CMake and pyosmium live in `.venv`. The runtime needs a C++17 compiler and
POSIX `fsync`.

```sh
python3 -m venv .venv
.venv/bin/pip install -r requirements-lock.txt
./scripts/download_map.sh
.venv/bin/python scripts/import_osm.py data/estonia.osm.pbf data/estonia.srg
./scripts/build_test.sh
./build/smartride_cli data/estonia.srg
```

`./scripts/build_test.sh` is the Release test entry point. A sanitizer argument
selects Debug instead. The benchmark entry point is:

```sh
scripts/run_benchmarks.sh
```

It configures Release with no sanitizer, runs CTest, then writes
`results/suite/`. If that directory already has files, the script exits
nonzero. `scripts/run_benchmarks.sh --force` replaces only `results/suite/`.
It does not download the map and does not overwrite the older result files.
Details and workload definitions are in [BENCHMARKS.md](BENCHMARKS.md).

## Project structure

| Path | Role |
|---|---|
| `include/smartride/`, `src/` | Graph, routing, matching, dispatcher, WAL, CLI |
| `tests/` | C++ correctness tests and `tests/test_import.py` |
| `benchmarks/` | Routing, CH, matching, dispatch, and WAL benchmarks |
| `scripts/build_test.sh` | Release or sanitized build plus tests |
| `scripts/run_benchmarks.sh` | Release suite into `results/suite/` |
| `scripts/download_map.sh`, `scripts/import_osm.py` | OSM PBF to `SRG1` |
| `docs/OSM.md`, `docs/ARCHITECTURE.md` | Map profile and routing invariants |
| `results/suite/` | Final measured suite |
| `results/*.txt` outside `suite/` | Earlier runs; do not cite as this suite |
| `PLAN.md`, `PROGRESS.md` | Checklist and milestone notes |

## Limitations and future work

Turn restrictions are counted and not enforced. There is no traffic, speed, or
time-dependent access model. Pickup matching is not a routed path. Greedy
batch cost depends on request order. Hungarian matching stops at 64 requests
and 512 drivers. The dispatcher was measured at 64 drivers, not 100,000, and
no million-request stress test was run.

Contraction-hierarchy memory in the suite is process RSS. ThreadSanitizer did
not complete on Apple arm64, so race freedom is not sanitizer-verified.
AddressSanitizer startup was left unverified. A crash during `fsync` itself,
and power loss after `write` but before `fsync`, were not reproduced.

`PLAN.md` still leaves approximation tradeoffs, the unchecked TSan clause, and
the million-request clause open. Plots, a technical report, and resume bullets
are not part of this repository yet.
