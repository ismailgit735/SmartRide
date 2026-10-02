#!/bin/sh
set -eu
cd "$(dirname "$0")/.."
mkdir -p results
{
  date -u
  sw_vers
  uname -a
  sysctl -n hw.model hw.ncpu hw.memsize
  df -h .
  clang++ --version
  .venv/bin/cmake --version
  git --version
  .venv/bin/python --version
} > results/environment.txt
