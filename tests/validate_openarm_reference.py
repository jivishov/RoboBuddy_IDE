"""Model-only invariants for the standalone OpenArm teaching scene.

These checks validate internal browser geometry consistency. They are not hardware
measurements and must never be used as hardware safety/certification evidence.
"""
import math

TABLE_TOP_MM = 320.0
FLASK_DIAMETER_MM = 30.4

def gap(command):
    c=max(-65.0,min(0.0,float(command)))
    return 6.0 + (-c / 65.0) * 66.0

def fk(j1,j2,j4):
    t1=math.radians(j2+10.0)
    t2=math.radians(j2+50.0-0.5*j4)
    radial=math.cos(t1)*160.0 + math.cos(t2)*220.0
    return {
        'x': -320.0 + radial*math.cos(math.radians(j1)),
        'y': 570.0 + math.sin(t1)*160.0 + math.sin(t2)*220.0,
        'z': -radial*math.sin(math.radians(j1)),
        'terminal_deg': math.degrees(t2),
    }

contact_overlap=max(0.0,FLASK_DIAMETER_MM-gap(-24.03))
pinch_overlap=max(0.0,FLASK_DIAMETER_MM-gap(-23.70))
assert contact_overlap < 0.15, contact_overlap
assert 0.15 <= pinch_overlap <= 0.35, pinch_overlap

approach=fk(10.0,-41.0,51.75)
flask_bottom_offset=approach['y']-TABLE_TOP_MM
place=fk(-18.0,-24.0,85.75)
hotplate_top=place['y']-flask_bottom_offset
transport=fk(-18.0,-20.0,93.75)
transport_bottom=transport['y']-flask_bottom_offset
assert hotplate_top > TABLE_TOP_MM
assert abs((place['y']-flask_bottom_offset)-hotplate_top) < 1e-9
assert transport_bottom-hotplate_top > 10.0

poses=[(-41.0,51.75),(-35.0,63.75),(-28.0,77.75),(-20.0,93.75),(-24.0,85.75)]
terminal=[fk(0,j2,j4)['terminal_deg'] for j2,j4 in poses]
assert max(terminal)-min(terminal) < 1e-9, terminal

print('OpenArm browser-reference geometry invariants: OK')
print('  first-contact overlap mm:',contact_overlap)
print('  pinch overlap mm:',pinch_overlap)
print('  hotplate top mm:',hotplate_top)
print('  transport clearance above hotplate mm:',transport_bottom-hotplate_top)
