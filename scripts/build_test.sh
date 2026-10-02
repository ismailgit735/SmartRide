#!/bin/sh
set -eu
cd "$(dirname "$0")/.."
BUILD=${1:-build}
SANITIZER=${2:-}
.venv/bin/cmake -S . -B "$BUILD" -DCMAKE_BUILD_TYPE=Release -DSANITIZER="$SANITIZER"
.venv/bin/cmake --build "$BUILD" -j 4
.venv/bin/ctest --test-dir "$BUILD" --output-on-failure
.venv/bin/python tests/test_import.py
