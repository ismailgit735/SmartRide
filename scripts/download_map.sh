#!/bin/sh
set -eu
cd "$(dirname "$0")/.."
mkdir -p data
curl -4 -L --fail --connect-timeout 20 --max-time 600 --retry 3 -o data/estonia.download.pbf https://download.geofabrik.de/europe/estonia-latest.osm.pbf
mv data/estonia.download.pbf data/estonia.osm.pbf
