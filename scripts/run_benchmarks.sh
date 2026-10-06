#!/bin/sh
# Release correctness tests and the measured benchmark suite.
# Writes only results/suite/. Existing results/*.txt evidence is left unchanged.
# Pass --force to replace a previous results/suite/ directory.
set -eu
cd "$(dirname "$0")/.."

FORCE=0
if [ "${1:-}" = "--force" ]; then
  FORCE=1
elif [ -n "${1:-}" ]; then
  echo "usage: scripts/run_benchmarks.sh [--force]" >&2
  exit 2
fi

OUT=results/suite
if [ -d "$OUT" ] && [ -n "$(find "$OUT" -mindepth 1 -print -quit)" ]; then
  if [ "$FORCE" -ne 1 ]; then
    echo "results/suite already contains files. Re-run with --force to replace only that directory." >&2
    exit 1
  fi
  rm -rf "$OUT"
fi
mkdir -p "$OUT"

echo "Configuring Release build"
.venv/bin/cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DSANITIZER=
.venv/bin/cmake --build build -j 4
.venv/bin/ctest --test-dir build --output-on-failure

MAP="${SMARTRIDE_SOURCE_DIR:-$PWD}/data/estonia.srg"
if [ ! -f "$MAP" ]; then
  echo "road graph not found: $MAP" >&2
  exit 1
fi

SHA=$(git rev-parse HEAD)
DIRTY=$(git status --porcelain)
if [ -z "$DIRTY" ]; then
  DIRTY_LABEL=clean
else
  DIRTY_LABEL=dirty
fi
TIMESTAMP=$(date -u '+%Y-%m-%dT%H:%M:%SZ')
FLAGS=$(.venv/bin/cmake -LA -N build | grep -E '^(CMAKE_BUILD_TYPE|CMAKE_CXX_COMPILER|CMAKE_CXX_FLAGS|CMAKE_CXX_FLAGS_RELEASE|CMAKE_CXX_STANDARD):' || true)

append_meta() {
  cat >> "$1" <<EOF

# Suite metadata
git_commit: $SHA
git_worktree: $DIRTY_LABEL
build_type: Release
timestamp_utc: $TIMESTAMP
platform: $(uname -srm)
warmup_policy: stated in this result file; warmup samples are excluded from reported latencies
cmake:
$FLAGS
EOF
}

finish() {
  append_meta "$1"
}

./build/routing_bench "$MAP" "$OUT/routing-benchmark.txt"
finish "$OUT/routing-benchmark.txt"

./build/ch_bench "$MAP" "$OUT/ch-benchmark.txt"
finish "$OUT/ch-benchmark.txt"

./build/matching_bench "$MAP" "$OUT/matching-benchmark.txt"
finish "$OUT/matching-benchmark.txt"

./build/batch_matching_bench "$MAP" > "$OUT/batch-matching-benchmark.txt"
finish "$OUT/batch-matching-benchmark.txt"

./build/hungarian_matching_bench "$MAP" > "$OUT/hungarian-matching-benchmark.txt"
finish "$OUT/hungarian-matching-benchmark.txt"

./build/dispatch_bench "$OUT/dispatch-benchmark.txt"
finish "$OUT/dispatch-benchmark.txt"

./build/wal_bench "$OUT/wal-benchmark.txt"
finish "$OUT/wal-benchmark.txt"
rm -rf "$OUT/wal-bench-tmp"

{
  echo "# Benchmark suite index"
  echo
  echo "git_commit: $SHA"
  echo "git_worktree: $DIRTY_LABEL"
  echo "build_type: Release"
  echo "timestamp_utc: $TIMESTAMP"
  echo "platform: $(uname -srm)"
  echo "map: $MAP"
  echo "ctest: passed before benchmarks"
  echo "historical_results: results/*.txt outside results/suite/ were not modified"
  echo
  echo "routing: results/suite/routing-benchmark.txt"
  echo "contraction_hierarchy: results/suite/ch-benchmark.txt"
  echo "nearest_driver: results/suite/matching-benchmark.txt"
  echo "greedy_batch: results/suite/batch-matching-benchmark.txt"
  echo "hungarian: results/suite/hungarian-matching-benchmark.txt"
  echo "dispatch: results/suite/dispatch-benchmark.txt"
  echo "wal: results/suite/wal-benchmark.txt"
  echo
  echo "cmake:"
  echo "$FLAGS"
} > "$OUT/index.txt"

echo "Benchmark suite wrote $OUT"
