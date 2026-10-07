#!/usr/bin/env python3
"""Build Asimov browser packages from the pinned, unmodified Menlo source.

No mesh simplification or inertial fitting. Physics retains source primitives;
lossless gzip STLs are presentation-only. Added torque actuators use URDF effort
limits; the PD controller and mounted/drop fixtures are RoboBuddy assumptions.
"""
import argparse, copy, gzip, hashlib, json, shutil
from pathlib import Path
import xml.etree.ElementTree as ET

REV = '732cc60dcb8f2b4fd26c3d7346b35f9b89c3cd47'
XML_BLOB = '4022cdfdc17b9596ef43a27bd6fd04149e088646'
ROOT = Path(__file__).resolve().parents[1]
def sha(b): return hashlib.sha256(b).hexdigest()
def vector(s, default): return list(map(float, (s or default).split()))
def json_write(p, obj): p.write_text(json.dumps(obj, indent=2) + '\n')

def generate(source: Path):
    xml_path = source / 'sim-model/xmls/asimov_1.xml'
    raw = xml_path.read_bytes()
    assert hashlib.sha1(b'blob ' + str(len(raw)).encode() + b'\0' + raw).hexdigest() == XML_BLOB, 'Unpinned Menlo MJCF'
    uraw = (source / 'sim-model/urdf/asimov_1.urdf').read_bytes()
    assert hashlib.sha1(b'blob ' + str(len(uraw)).encode() + b'\0' + uraw).hexdigest() == '887050863f708f6da6bfc21065a7019172c258c9', 'Unpinned Menlo URDF'
    original = ET.fromstring(raw); urdf = ET.fromstring(uraw)
    limits = {j.get('name'): j.find('limit') for j in urdf.findall('joint') if j.find('limit') is not None}
    joints = []
    for j in original.findall('.//worldbody//joint'):
        name = j.get('name'); u = limits[name]
        joints.append(dict(id=name, rangeRad=vector(j.get('range'), '0 0'), axis=vector(j.get('axis'), '0 0 1'),
                           referenceRad=float(j.get('ref', 0)), effortLimitNm=float(u.get('effort')),
                           velocityLimitRadS=float(u.get('velocity')), armature=float(j.get('armature', 0))))
    assert len(joints) == 23 and len(set(j['id'] for j in joints)) == 23
    bodies = [b.get('name') for b in original.findall('.//worldbody//body')]
    assert len(bodies) == 26
    out = ROOT / 'models/asimov'; (out/'source').mkdir(parents=True, exist_ok=True); (out/'visual').mkdir(exist_ok=True)
    for name in ('HARDWARE-LICENSE.txt', 'SOFTWARE-LICENSE.txt'):
        shutil.copyfile(source/name, out/'source'/name)
    (out/'source/asimov_1.xml').write_bytes(raw); (out/'source/asimov_1.urdf').write_bytes(uraw)
    meshes = {m.get('name'): m for m in original.findall('./asset/mesh')}
    files = {}
    for m in meshes.values():
        file = m.get('file')
        if file in files: continue
        data = (source/'sim-model/assets/meshes'/file).read_bytes()
        compressed = gzip.compress(data, compresslevel=9, mtime=0)
        asset = f'models/asimov/visual/{file}.gz'; (ROOT/asset).write_bytes(compressed)
        files[file] = dict(asset=asset, sha256=sha(compressed), rawSha256=sha(data), bytes=len(compressed), rawBytes=len(data))
    visual = []
    for b in original.findall('.//worldbody//body'):
        for g in b.findall('geom'):
            if g.get('class') == 'visual':
                m = meshes[g.get('mesh')]
                visual.append(dict(body=b.get('name'), file=m.get('file'), positionM=vector(g.get('pos'), '0 0 0'),
                                   quaternionWxyz=vector(g.get('quat'), '1 0 0 0'), scale=vector(m.get('scale'), '1 1 1')))
    json_write(out/'visual/manifest.json', dict(revision=REV, meshes=files, instances=visual))
    package_hashes = {}
    for variant, height, mounted in [('freebase', .630, False), ('mounted', 1.05, True), ('drop', .95, False)]:
        x = copy.deepcopy(original); x.set('model', f'robobuddy-asimov-{variant}-v1')
        x.find('compiler').attrib.pop('meshdir', None)
        # These source visual geoms are zero density, zero contact. Every collider remains.
        for parent in x.iter():
            for child in list(parent):
                if child.tag == 'asset' or (child.tag == 'geom' and child.get('class') == 'visual'):
                    parent.remove(child)
        for e in x.iter(): e.attrib.pop('material', None)
        pelvis = x.find('./worldbody/body'); pelvis.set('pos', f'0 0 {height}')
        if mounted:
            pelvis.remove(pelvis.find('freejoint'))
            # Source-adjacent pairs automatically filtered in free-base model must also be
            # excluded when MuJoCo welds the parent to world (mounted fixture only).
            contact = x.find('contact')
            existing = {frozenset((e.get('body1'), e.get('body2'))) for e in contact}
            for child in pelvis.findall('body'):
                pair = ('pelvis_link', child.get('name'))
                if frozenset(pair) not in existing: ET.SubElement(contact, 'exclude', body1=pair[0], body2=pair[1])
            ET.SubElement(x.find('worldbody'), 'geom', name='asimov_declared_mount', type='box',
                          pos='-0.17 0 0.50', size='0.025 0.06 0.50', contype='0', conaffinity='0', rgba='.2 .3 .4 1')
        actuator = ET.SubElement(x, 'actuator')
        for j, spec in zip(x.findall('.//worldbody//joint'), joints):
            f = spec['effortLimitNm']; rng = f'{-f:g} {f:g}'
            j.set('actuatorfrcrange', rng)
            ET.SubElement(actuator, 'motor', name=spec['id'], joint=spec['id'], gear='1', ctrlrange=rng, forcerange=rng)
        positions = ([] if mounted else [0, 0, height, 1, 0, 0, 0]) + [j['referenceRad'] for j in joints]
        ET.SubElement(ET.SubElement(x,'keyframe'), 'key', name='declared_initial', qpos=' '.join(f'{v:.15g}' for v in positions))
        ET.indent(x, space='  ')
        data = ET.tostring(x, encoding='utf-8', xml_declaration=True) + b'\n'
        (out/f'{variant}.xml').write_bytes(data); package_hashes[variant] = sha(data)
    metadata = dict(revision=REV, sourceXmlSha256=sha(raw), sourceUrdfSha256=sha(uraw),
                    totalMassKg=sum(float(e.get('mass')) for e in original.findall('.//inertial')),
                    joints=joints, bodies=bodies, modelHashes=package_hashes,
                    visualManifestSha256=sha((out/'visual/manifest.json').read_bytes()),
                    exclusions=[dict(e.attrib) for e in original.findall('./contact/exclude')],
                    physicsOptions=dict(original.find('option').attrib))
    json_write(out/'source/audit.json', metadata)
    (ROOT/'src/physics/asimov-generated.js').write_text('// Generated by scripts/generate_asimov_models.py; do not hand edit.\n'
        + 'export const ASIMOV_SOURCE = Object.freeze(' + json.dumps(metadata, separators=(',',':')) + ');\n')
    print(json.dumps(dict(models=package_hashes, joints=len(joints), bodies=len(bodies), meshes=len(files), massKg=metadata['totalMassKg'])))

if __name__ == '__main__':
    p=argparse.ArgumentParser(); p.add_argument('--source', type=Path, required=True); generate(p.parse_args().source)
