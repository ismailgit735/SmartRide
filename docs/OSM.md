# OpenStreetMap data profile

© [OpenStreetMap contributors](https://www.openstreetmap.org/copyright),
[ODbL 1.0](https://opendatacommons.org/licenses/odbl/1-0/).
Extract provider: [Geofabrik](https://download.geofabrik.de/europe/estonia.html).
Generated road graphs remain ODbL-derived data. Code licensing is separate.
The raw PBF and graph are local artifacts excluded from Git because of size.
`data/estonia.json` records input/output SHA256, actual counts, and import time.
The latest URL changes: retain the downloaded PBF for bit-identical reproduction.

Reproduce with `scripts/download_map.sh`, then
`.venv/bin/python scripts/import_osm.py data/estonia.osm.pbf data/estonia.srg`.
The established [pyosmium/libosmium parser](https://docs.osmcode.org/pyosmium/latest/reference/Handler-Processing/)
resolves OSM node locations; Python exports a text SRG1 graph for the C++ engine.
We retain every accepted way segment, including shape nodes; no edge inflation
or synthetic duplication is used to reach the edge target.

Supported highway classes: motorway, trunk, primary, secondary, tertiary and their
links; unclassified, residential, living_street, service. No tracks, footways,
ferries, construction, or area ways. Access precedence is motorcar > motor_vehicle
> vehicle > access. At each level directional access overrides nondirectional.
Only absent/yes/permissive/designated/official access is accepted. Destination,
customers, delivery, private and other values are conservatively excluded.
[One-way tags](https://wiki.openstreetmap.org/wiki/Key:oneway): yes/1/true,
-1 (reverse), no/0/false; motorway and roundabout/circular imply forward only
unless overridden. Motorcar/motor_vehicle-specific oneway overrides generic tags.
Unknown/reversible one-way and any conditional way tags are excluded.

Weights are nonnegative haversine segment lengths in meters, earth radius
6,371,000 m. Routes minimize distance, not travel time; no traffic or speed model.
Points use a 59° equirectangular projection for the spatial index. Routing
heuristics are scaled by the minimum edge-weight/projected-length ratio so
triangle inequality gives a lower bound even with projection distortion.

Unsupported: relation-based turn restrictions (counted in metadata), barriers
and access restrictions on nodes, lane restrictions, time-dependent access,
vehicle size/weight rules, toll preferences, country-specific implicit access,
and real-time closures. This is a research routing profile, not a navigation
product. Disconnected roads are retained; unreachable routes return infinity.
