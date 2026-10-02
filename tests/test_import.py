import sys
from pathlib import Path
import tempfile
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from import_osm import directions, Importer

def check(tags, expected):
    assert directions(dict(highway='residential', **tags)) == expected
check({}, (True, True))
check({'oneway': 'yes'}, (True, False))
check({'oneway': '-1'}, (False, True))
check({'access': 'private'}, (False, False))
check({'access': 'no', 'motorcar': 'yes'}, (True, True))
check({'motor_vehicle:backward': 'no'}, (True, False))
check({'junction': 'roundabout'}, (True, False))
check({'junction': 'roundabout', 'oneway': 'no'}, (True, True))
check({'oneway:conditional': 'yes @ (Mo-Fr)'}, (False, False))
check({'oneway': 'reversible'}, (False, False))
check({'access': 'destination'}, (False, False))
assert directions({'highway':'motorway'}) == (True, False)
assert directions({'highway':'footway'}) == (False, False)
with tempfile.TemporaryDirectory() as d:
    p = Path(d) / 'tiny.osm'
    p.write_text('''<osm version="0.6"><node id="1" lat="59" lon="24"/><node id="2" lat="59" lon="24.001"/><node id="3" lat="59" lon="24.002"/><way id="10"><nd ref="1"/><nd ref="2"/><nd ref="3"/><tag k="highway" v="residential"/><tag k="oneway" v="-1"/></way></osm>''')
    with (Path(d)/'edges').open('w+') as f:
        h=Importer(f); h.apply_file(str(p), locations=True)
        f.seek(0); lines=f.readlines()
        assert len(lines)==2 and lines[0].startswith('1 0 ') and lines[1].startswith('2 1 ')
        assert 50 < float(lines[0].split()[2]) < 60
print('OSM profile and real parser fixture tests passed')
