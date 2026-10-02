#!/usr/bin/env python3
"""Streaming OSM parsing via pyosmium/libosmium; emit a C++ SRG1 graph."""
import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import time
import osmium

ROADS = {'motorway', 'trunk', 'primary', 'secondary', 'tertiary', 'unclassified',
         'residential', 'living_street', 'service', 'motorway_link', 'trunk_link',
         'primary_link', 'secondary_link', 'tertiary_link'}
ALLOWED = {'yes', 'permissive', 'designated', 'official'}
R = 6371000.0

def access(tags, direction=None):
    # Most specific vehicle mode wins; direction wins at the same specificity.
    value = None
    for key in ('access', 'vehicle', 'motor_vehicle', 'motorcar'):
        value = tags.get(key, value)
        if direction:
            value = tags.get(key + ':' + direction, value)
    return value is None or value in ALLOWED

def directions(tags):
    if tags.get('highway') not in ROADS or tags.get('area') == 'yes':
        return False, False
    if any(':conditional' in k for k in tags):
        return False, False  # exclude time-dependent roads rather than guess
    ow = tags.get('oneway:motorcar', tags.get('oneway:motor_vehicle', tags.get('oneway')))
    if ow is None:
        ow = 'yes' if tags.get('highway') == 'motorway' or tags.get('junction') in ('roundabout', 'circular') else 'no'
    if ow not in ('yes', '1', 'true', '-1', 'no', '0', 'false'):
        return False, False
    return ow != '-1' and access(tags, 'forward'), ow not in ('yes', '1', 'true') and access(tags, 'backward')

def haversine(a, b):
    lat1, lon1, lat2, lon2 = map(math.radians, (*a, *b))
    h = math.sin((lat2-lat1)/2)**2 + math.cos(lat1)*math.cos(lat2)*math.sin((lon2-lon1)/2)**2
    return 2*R*math.asin(min(1.0, math.sqrt(h)))

class Importer(osmium.SimpleHandler):
    def __init__(self, spool):
        super().__init__()
        self.nodes = {}
        self.points = []
        self.spool = spool
        self.stats = Counter()

    def way(self, way):
        tags = dict(way.tags)
        forward, backward = directions(tags)
        if not (forward or backward):
            return
        self.stats['accepted_ways'] += 1
        for a, b in zip(way.nodes, list(way.nodes)[1:]):
            if not a.location.valid() or not b.location.valid():
                self.stats['missing_location_segments'] += 1
                continue
            ids = []
            coords = []
            for n in (a, b):
                if n.ref not in self.nodes:
                    self.nodes[n.ref] = len(self.points)
                    # Equirectangular coordinates for spatial indexing; routing
                    # uses exact haversine segment lengths, not this projection.
                    self.points.append((R*math.radians(n.lon)*math.cos(math.radians(59)), R*math.radians(n.lat)))
                ids.append(self.nodes[n.ref])
                coords.append((n.lat, n.lon))
            weight = haversine(*coords)
            if ids[0] == ids[1]:
                continue
            if forward:
                self.spool.write(f'{ids[0]} {ids[1]} {weight:.17g}\n')
                self.stats['directed_edges'] += 1
            if backward:
                self.spool.write(f'{ids[1]} {ids[0]} {weight:.17g}\n')
                self.stats['directed_edges'] += 1

    def relation(self, relation):
        if relation.tags.get('type') == 'restriction':
            self.stats['unsupported_turn_restriction_relations'] += 1

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('input', type=Path)
    ap.add_argument('output', type=Path)
    args = ap.parse_args()
    start = time.perf_counter()
    tmp = args.output.with_suffix('.tmp')
    with tmp.open('w+') as spool:
        handler = Importer(spool)
        handler.apply_file(str(args.input), locations=True, idx='flex_mem')
        spool.seek(0)
        with args.output.open('w') as out:
            out.write(f'SRG1 {len(handler.points)} {handler.stats["directed_edges"]}\n')
            for x, y in handler.points:
                out.write(f'{x:.17g} {y:.17g}\n')
            import shutil
            shutil.copyfileobj(spool, out)
    tmp.unlink()
    def sha(path):
        with path.open('rb') as f:
            return hashlib.file_digest(f, 'sha256').hexdigest()
    metadata = dict(handler.stats, nodes=len(handler.points), seconds=time.perf_counter()-start,
                    input=str(args.input), input_sha256=sha(args.input), output_sha256=sha(args.output),
                    attribution='© OpenStreetMap contributors, ODbL 1.0; extract by Geofabrik',
                    source='https://download.geofabrik.de/europe/estonia-latest.osm.pbf')
    args.output.with_suffix('.json').write_text(json.dumps(metadata, indent=2)+'\n')
    print(json.dumps(metadata, indent=2))
if __name__ == '__main__':
    main()
