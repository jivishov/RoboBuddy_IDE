#!/usr/bin/env python3
"""Prepare the Unitree G1 29-DoF physical collision assets.

The pinned source model `unitreerobotics/unitree_ros@dd4fa686.../robots/g1_description/
g1_29dof.xml` uses full STL link meshes as its collision geometry. MuJoCo collides mesh
geoms through their convex hulls, so shipping the convex hull of each source collision
mesh is collision-identical, not an approximation, while removing 7.5 MB of triangles the
physics never uses. `tests/validate_unitree_g1_assets.mjs` and the native reference runner
both re-assert that identity.

Modes
  (default)  offline: verify the committed assets against models/unitree_g1/source/MESH_MANIFEST.json
  --fetch    download the pinned upstream STLs, re-derive the hulls, and compare
  --write    with --fetch, rewrite the committed assets and the manifest
"""
from __future__ import annotations

import argparse
import hashlib
import json
import re
import struct
import sys
import urllib.request
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
SOURCE_DIR = REPO / "models" / "unitree_g1" / "source"
ASSET_DIR = REPO / "models" / "unitree_g1" / "assets"
MANIFEST = SOURCE_DIR / "MESH_MANIFEST.json"

UPSTREAM_REPO = "unitreerobotics/unitree_ros"
UPSTREAM_REVISION = "dd4fa6866e523ad61324f658d63736e4eda3a6e4"
UPSTREAM_MESH_PATH = "robots/g1_description/meshes"
UPSTREAM_RAW = "https://raw.githubusercontent.com"

STL_HEADER = b"RoboBuddy G1 29-DoF collision hull: convex hull of pinned unitree_ros mesh"


def sha256(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def read_binary_stl(data: bytes) -> "list[list[list[float]]]":
    if data[:5].lower() == b"solid" and b"facet" in data[:512]:
        raise SystemExit("ASCII STL is not supported; the pinned G1 meshes are binary STL")
    count = struct.unpack("<I", data[80:84])[0]
    if len(data) != 84 + 50 * count:
        raise SystemExit("malformed binary STL")
    out = []
    for index in range(count):
        base = 84 + 50 * index + 12
        tri = struct.unpack("<9f", data[base:base + 36])
        out.append([list(tri[0:3]), list(tri[3:6]), list(tri[6:9])])
    return out


def write_binary_stl(triangles) -> bytes:
    import numpy as np

    out = bytearray(STL_HEADER[:80].ljust(80, b"\0"))
    out += struct.pack("<I", len(triangles))
    for tri in triangles:
        tri = np.asarray(tri, dtype=float)
        normal = np.cross(tri[1] - tri[0], tri[2] - tri[0])
        length = float(np.linalg.norm(normal))
        normal = normal / length if length > 0 else np.zeros(3)
        out += struct.pack("<3f", *normal.astype("<f4"))
        for vertex in tri:
            out += struct.pack("<3f", *vertex.astype("<f4"))
        out += b"\0\0"
    return bytes(out)


def collision_mesh_names() -> "list[str]":
    """The mesh files the pinned source model actually uses as collision geometry.

    A source geom is collision geometry unless it explicitly opts out with
    contype="0" conaffinity="0", which the source uses for its group="1" visual copies.
    """
    xml = (SOURCE_DIR / "g1_29dof.xml").read_text(encoding="utf-8")
    mesh_files = dict(re.findall(r'<mesh\s+name="([^"]+)"\s+file="([^"]+)"', xml))
    names = []
    for geom in re.findall(r"<geom\b[^>]*/>", xml):
        if 'type="mesh"' not in geom:
            continue
        if 'contype="0"' in geom and 'conaffinity="0"' in geom:
            continue
        match = re.search(r'mesh="([^"]+)"', geom)
        if match and match.group(1) not in names:
            names.append(match.group(1))
    missing = [name for name in names if name not in mesh_files]
    if missing:
        raise SystemExit(f"source model references undeclared meshes: {missing}")
    return sorted(mesh_files[name] for name in names)


def convex_hull(data: bytes):
    import numpy as np
    from scipy.spatial import ConvexHull

    triangles = np.asarray(read_binary_stl(data), dtype=float)
    points = np.unique(np.round(triangles.reshape(-1, 3), 9), axis=0)
    vertices = points[ConvexHull(points).vertices]
    hull = ConvexHull(vertices)
    faces = vertices[hull.simplices]
    centre = vertices.mean(axis=0)
    for index in range(faces.shape[0]):
        normal = np.cross(faces[index, 1] - faces[index, 0], faces[index, 2] - faces[index, 0])
        if float(np.dot(normal, faces[index].mean(axis=0) - centre)) < 0:
            faces[index] = faces[index][[0, 2, 1]]
    return faces, vertices, float(hull.volume)


def fetch(name: str) -> bytes:
    url = f"{UPSTREAM_RAW}/{UPSTREAM_REPO}/{UPSTREAM_REVISION}/{UPSTREAM_MESH_PATH}/{name}"
    with urllib.request.urlopen(url, timeout=120) as response:
        if response.status != 200:
            raise SystemExit(f"{name}: upstream returned HTTP {response.status}")
        return response.read()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--fetch", action="store_true", help="download the pinned upstream STLs and re-derive the hulls")
    parser.add_argument("--write", action="store_true", help="with --fetch, rewrite the committed assets and manifest")
    args = parser.parse_args()

    names = collision_mesh_names()
    if not args.fetch:
        if not MANIFEST.exists():
            print("no committed manifest; run with --fetch --write", file=sys.stderr)
            return 1
        manifest = json.loads(MANIFEST.read_text(encoding="utf-8"))
        if manifest["upstream"]["revision"] != UPSTREAM_REVISION:
            print("manifest pins a different upstream revision", file=sys.stderr)
            return 1
        recorded = {row["mesh"]: row for row in manifest["meshes"]}
        if sorted(recorded) != names:
            print(f"manifest mesh set differs from the source collision set: {sorted(set(recorded) ^ set(names))}", file=sys.stderr)
            return 1
        failures = []
        for name in names:
            path = ASSET_DIR / name
            if not path.exists():
                failures.append(f"{name}: missing committed hull asset")
                continue
            digest = sha256(path.read_bytes())
            if digest != recorded[name]["hullSha256"]:
                failures.append(f"{name}: hull SHA-256 {digest} != manifest {recorded[name]['hullSha256']}")
        stray = sorted(p.name for p in ASSET_DIR.glob("*.STL") if p.name not in recorded)
        if stray:
            failures.append(f"unmanifested asset files present: {stray}")
        for failure in failures:
            print(failure, file=sys.stderr)
        if failures:
            return 1
        print(f"unitree_g1 assets verified: {len(names)} collision hulls match MESH_MANIFEST.json")
        return 0

    ASSET_DIR.mkdir(parents=True, exist_ok=True)
    rows = []
    mismatched = []
    for name in names:
        data = fetch(name)
        faces, vertices, volume = convex_hull(data)
        hull_bytes = write_binary_stl(faces)
        row = {
            "mesh": name,
            "sourceSha256": sha256(data),
            "sourceBytes": len(data),
            "sourceTriangles": (len(data) - 84) // 50,
            "hullSha256": sha256(hull_bytes),
            "hullBytes": len(hull_bytes),
            "hullVertices": int(len(vertices)),
            "hullTriangles": int(faces.shape[0]),
            "hullVolumeM3": round(volume, 12),
        }
        rows.append(row)
        target = ASSET_DIR / name
        if args.write:
            target.write_bytes(hull_bytes)
        elif not target.exists() or sha256(target.read_bytes()) != row["hullSha256"]:
            mismatched.append(name)
        print(f"{name:34s} {row['sourceTriangles']:6d} tris -> {row['hullTriangles']:5d}   {row['sourceBytes']:8d} B -> {row['hullBytes']:7d} B")

    if args.write:
        MANIFEST.write_text(json.dumps({
            "schema": "robobuddy.unitree-g1.mesh-manifest.v1",
            "upstream": {"repository": UPSTREAM_REPO, "revision": UPSTREAM_REVISION, "path": UPSTREAM_MESH_PATH, "license": "BSD-3-Clause"},
            "derivation": "convex hull of each source collision mesh; MuJoCo collides mesh geoms through their convex hull, so this is collision-identical to the source model",
            "generator": "scripts/prepare_unitree_g1_assets.py",
            "meshes": rows,
        }, indent=1) + "\n", encoding="utf-8")
        print(f"wrote {len(rows)} hull assets and {MANIFEST.relative_to(REPO)}")
        return 0

    if mismatched:
        print(f"committed assets differ from the freshly derived hulls: {mismatched}", file=sys.stderr)
        return 1
    print(f"re-derived {len(rows)} hulls from upstream; all committed assets match")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
