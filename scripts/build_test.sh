#!/bin/sh
set -eu
cd "$(dirname "$0")/.."
BUILD=${1:-build}
SANITIZER=${2:-}
if [ -n "$SANITIZER" ]; then
  BUILD_TYPE=Debug
else
  BUILD_TYPE=Release
fi
echo "Configuring ${BUILD}: CMAKE_BUILD_TYPE=${BUILD_TYPE} SANITIZER=${SANITIZER:-none}"
.venv/bin/cmake -S . -B "$BUILD" -DCMAKE_BUILD_TYPE="$BUILD_TYPE" -DSANITIZER="$SANITIZER"
.venv/bin/cmake --build "$BUILD" -j 4
.venv/bin/ctest --test-dir "$BUILD" --output-on-failure
.venv/bin/python tests/test_import.py
