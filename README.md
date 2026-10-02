# SmartRide

C++17 ride matching and directed road routing engine. No GUI.

## Build and baseline demo

```sh
python3 -m venv .venv
.venv/bin/pip install -r requirements-lock.txt
./scripts/build_test.sh
./build/smartride_cli
```

CMake and Python tooling live in the project virtual environment. Runtime uses
C++17 and the standard library with native POSIX persistence on macOS/Linux.
See [PLAN.md](PLAN.md) for all requirements and [PROGRESS.md](PROGRESS.md) for
verified milestones. [OSM profile](docs/OSM.md) documents map semantics and limits.

```sh
./scripts/download_map.sh
.venv/bin/python scripts/import_osm.py data/estonia.osm.pbf data/estonia.srg
./build/smartride_cli data/estonia.srg
```
