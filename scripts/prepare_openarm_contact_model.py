#!/usr/bin/env python3
"""Build the repository-local OpenArm contact/render package from pinned Enactic sources.

The convex hull of EACH upstream collision component is retained separately. The
same vertices/triangles feed MJCF and Three.js; no visual-only finger offset or
invisible grip extension is used. Requires numpy/scipy; native MuJoCo is not
needed for generation. --upstream-dir allows fully offline regeneration.
"""
from __future__ import annotations
import argparse
import hashlib
import json
import struct
import urllib.request
import xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
from scipy.spatial import ConvexHull

ROOT = Path(__file__).resolve().parents[1]
REVISION = 'a8c979629f2591ad035d99d338ce114969e6cddc'
SOURCE_BLOB = '0bd77d3bf7e0a5f3d2361fdf5e8328d00d0b2cc9'
BASELINE_SHA = '960ecf32c0aa7c8b2b016c6f28a7a8afe8147ce6cb1cdfd9b91f550cd4fc27dc'


def digest(value: bytes) -> str:
    return hashlib.sha256(value).hexdigest()


def read_source(path: str, directory: Path | None) -> bytes:
    if directory:
        return (directory / path).read_bytes()
    url = f'https://raw.githubusercontent.com/enactic/openarm_mujoco/{REVISION}/v2/{path}'
    with urllib.request.urlopen(url, timeout=30) as response:
        return response.read()


def numbers(value: str | None, default: str) -> list[float]:
    return [float(x) for x in (value or default).split()]


def build(directory: Path | None = None) -> None:
    baseline = (ROOT / 'models/openarm_v2/source/primitive-baseline.xml').read_bytes()
    if digest(baseline) != BASELINE_SHA:
        raise ValueError('Immutable primitive baseline hash changed')
    root = ET.fromstring(baseline)
    source_bytes = read_source('openarm_bimanual.xml', directory)
    blob = hashlib.sha1(f'blob {len(source_bytes)}\0'.encode() + source_bytes).hexdigest()
    if blob != SOURCE_BLOB:
        raise ValueError('Pinned OpenArm source XML blob mismatch')
    source = ET.fromstring(source_bytes)
    source_meshes = {a.get('name'): a for a in source.findall('./asset/mesh')}
    source_bodies = {b.get('name'): b for b in source.iter('body')}
    assets = ET.Element('asset')
    root.insert(3, assets)
    meshes, inputs = {}, {'openarm_bimanual.xml': digest(source_bytes)}
    for body in root.iter('body'):
        original = source_bodies.get(body.get('name'))
        if original is None:
            continue
        for geom in body.findall('geom'):
            body.remove(geom)
        for upstream_geom in original.findall('geom'):
            if upstream_geom.get('class') not in ('collision', 'fingertip'):
                continue
            geom = ET.fromstring(ET.tostring(upstream_geom))
            geom.attrib.pop('material', None)
            finger = upstream_geom.get('class') == 'fingertip'
            geom.set('class', 'left_fingertip' if finger else 'robot_left_collision')
            # Adjacent-body filtering remains MuJoCo's standard mechanism. Do not
            # suppress all non-adjacent same-arm collisions through bit masks.
            geom.set('contype', '3')
            geom.set('conaffinity', '3')
            geom.set('rgba', '0.19 0.23 0.28 1' if finger or 'ee_base' in geom.get('name') else '0.44 0.48 0.53 1')
            mesh = source_meshes[geom.get('mesh')]
            data = read_source('assets/' + mesh.get('file'), directory)
            inputs['assets/' + mesh.get('file')] = digest(data)
            count = struct.unpack_from('<I', data, 80)[0]
            if len(data) != 84 + count * 50:
                raise ValueError(f'Invalid binary STL length: {mesh.get("file")}')
            dtype = np.dtype([('normal', '<f4', (3,)), ('vertices', '<f4', (3, 3)), ('attribute', '<u2')])
            points = np.frombuffer(data, dtype=dtype, offset=84, count=count)['vertices'].reshape(-1, 3).astype(float)
            points = np.unique(points, axis=0) * np.array(numbers(mesh.get('scale'), '1 1 1'))
            hull = ConvexHull(points)
            vertices = points[hull.vertices]
            mapping = {old: new for new, old in enumerate(hull.vertices)}
            triangles = []
            for face, equation in zip(hull.simplices, hull.equations):
                f = list(face)
                if np.dot(np.cross(points[f[1]] - points[f[0]], points[f[2]] - points[f[0]]), equation[:3]) < 0:
                    f[1], f[2] = f[2], f[1]
                triangles.extend(mapping[v] for v in f)
            flat_vertices = [float(f'{v:.10g}') for v in vertices.flat]
            ET.SubElement(assets, 'mesh', name=geom.get('mesh'), vertex=' '.join(map(str, flat_vertices)), face=' '.join(map(str, triangles)))
            meshes[geom.get('mesh')] = {'vertices': flat_vertices, 'indices': triangles}
            body.append(geom)
    # The source upper torque limit is 7 Nm. This tighter, declared operating
    # limit is a conservative SIMULATION setting for the lightweight dry task.
    for actuator in root.findall('./actuator/position'):
        if 'finger' in actuator.get('name', ''):
            actuator.set('forcerange', '-1.2 1.2')
    world = root.find('worldbody')
    named = {g.get('name'): g for g in world.findall('geom')}
    named['cell_table'].set('pos', '0.41 0 0.9925')
    named['cell_table'].set('size', '0.41 0.55 0.0125')  # top stays 1.005 m
    for name in ('left_source_support', 'right_source_support'):
        p = numbers(named[name].get('pos'), '0 0 0'); p[0] = .55
        named[name].set('pos', ' '.join(map(str, p)))
    for name in ('left_hotplate', 'right_ring_post', 'right_ring_gauze'):
        p = numbers(named[name].get('pos'), '0 0 0'); p[0] = .67
        named[name].set('pos', ' '.join(map(str, p)))
    for body in world.findall('body'):
        if body.get('name') in ('flask', 'beaker'):
            p = numbers(body.get('pos'), '0 0 0'); p[0] = .55
            body.set('pos', ' '.join(map(str, p)))
    for x in (.045, .775):
        for y in (-.505, .505):
            ET.SubElement(world, 'geom', name=f'table_leg_{int(x*1000)}_{int(y*1000)}', type='box', pos=f'{x} {y} .49', size='.018 .018 .49', **{'class': 'fixture'}, rgba='.24 .29 .34 1')
    ET.SubElement(world, 'geom', name='mount_column', type='box', pos='.17 0 1.142', size='.035 .04 .137', **{'class': 'fixture'}, rgba='.24 .29 .34 1')
    ET.SubElement(world, 'geom', name='mount_foot', type='box', pos='.17 0 1.010', size='.070 .070 .005', **{'class': 'fixture'}, rgba='.24 .29 .34 1')
    ET.SubElement(world, 'geom', name='right_ring_bracket', type='box', pos='.67 -.19725 1.069', size='.005 .040 .003', **{'class': 'fixture'}, rgba='.30 .34 .38 1')
    # Tool points are explicit in the physical EE frame. They are references,
    # not object attachments and not a guarantee of contact at any aperture.
    for side in ('left', 'right'):
        body = root.find(f'.//body[@name="openarm_{side}_ee_base_link"]')
        ET.SubElement(body, 'site', name=f'{side}_pinch_reference', pos='-.00143 0 -.153', size='.003')
    root.set('model', 'robobuddy_openarm_v2_contact_v3')
    ET.indent(root)
    xml = b'<?xml version="1.0" encoding="utf-8"?>\n<!-- OpenArm-derived robot: Apache-2.0. See PROVENANCE.md and licenses/openarm-v2-apache-2.0.txt. Generated by scripts/prepare_openarm_contact_model.py. -->\n' + ET.tostring(root) + b'\n'
    output = ROOT / 'models/openarm_v2/manipulation.xml'
    output.write_bytes(xml)
    geoms = []
    for parent in root.iter():
        if parent.tag not in ('worldbody', 'body'):
            continue
        for geom in parent.findall('geom'):
            geoms.append({'id': geom.get('name'), 'bodyId': parent.get('name', 'world'), 'type': geom.get('type'), 'positionM': numbers(geom.get('pos'), '0 0 0'), 'quaternionWxyz': numbers(geom.get('quat'), '1 0 0 0'), 'sizeM': numbers(geom.get('size'), ''), 'mesh': geom.get('mesh'), 'rgba': numbers(geom.get('rgba'), '.40 .46 .50 1')})
    package = {'schemaVersion': 'robobuddy.openarm.geometry.v1', 'modelSha256': digest(xml), 'source': {'repository': 'enactic/openarm_mujoco', 'revision': REVISION, 'sourceXmlBlob': SOURCE_BLOB, 'license': 'Apache-2.0', 'method': 'convex hull per upstream collision component; mirrored source scaling baked into both physics and rendering', 'inputSha256': dict(sorted(inputs.items()))}, 'meshes': meshes, 'geoms': geoms}
    geometry = (json.dumps(package, separators=(',', ':'), sort_keys=True) + '\n').encode()
    (ROOT / 'models/openarm_v2/geometry.json').write_bytes(geometry)
    (ROOT / 'src/physics/openarm-generated.js').write_text('// Generated by scripts/prepare_openarm_contact_model.py.\n' + f"export const OPENARM_MODEL_SHA256 = '{digest(xml)}';\nexport const OPENARM_GEOMETRY_SHA256 = '{digest(geometry)}';\n")
    print(json.dumps({'modelSha256': digest(xml), 'geometrySha256': digest(geometry), 'meshComponents': len(meshes), 'physicalGeoms': len(geoms)}))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--upstream-dir', type=Path, help='Directory containing pinned v2/openarm_bimanual.xml and assets/')
    args = parser.parse_args()
    build(args.upstream_dir)
