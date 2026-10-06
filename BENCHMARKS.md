# Reproducing the benchmark suite

The measured suite is `scripts/run_benchmarks.sh`. It configures a Release build with no sanitizer, runs CTest, then runs the benchmarks. Outputs go to `results/suite/`. The script returns nonzero if configure, a test, or a benchmark fails.

```sh
scripts/run_benchmarks.sh
```

If `results/suite/` already contains files, the script stops. Replace only that directory with:

```sh
scripts/run_benchmarks.sh --force
```

Files already in `results/` outside `results/suite/` are not overwritten. Those include `routing-benchmark.txt`, `ch-benchmark.txt`, `matching-benchmark.txt`, `batch-matching-benchmark.txt`, and `hungarian-matching-benchmark.txt`.

The road graph is `data/estonia.srg`. The script does not download it.

## What each result measures

Routing and contraction-hierarchy queries use 100 distinct node pairs from `mt19937_64` seed 918273. Correctness against Dijkstra runs before timing. Latency is the query call only. The first 10 pairs are warmup and are excluded. Percentile index is `floor(p * n)`.

Nearest-driver, greedy batch, and Hungarian benchmarks use the workloads already implemented in their programs: driver seed 42, request/query seed 1234567, and the driver and batch counts printed in each result. A mismatch aborts before a successful result is kept.

Dispatch uses synthetic coordinates, not the road graph. Serial latency is one `submit()` through `drain()` on an idle one-worker dispatcher. It includes queue wait and matching. It does not include `release()` or WAL `fsync`. Throughput rounds time the same submit-to-drain boundary for 8 rounds of 64 requests on 64 drivers, at 1, 4, and 8 workers. The contention row is 8 workers, 4 drivers, and 64 requests that share one pickup. Duplicate driver assignments abort the run.

WAL append latency is one `append_assignment` call, which writes one record and `fsync`s it. Dispatcher WAL latency is `submit()` through `drain()` and includes queue wait, matching, and that per-record `fsync`. Replay is checked before the result file is written.

Each file under `results/suite/` ends with the git commit, whether the worktree was clean, the Release build type, the UTC timestamp, the platform, and the CMake compiler flags captured for that run. Warmup policy is stated in the result itself.

This suite does not run a million-request stress test.
