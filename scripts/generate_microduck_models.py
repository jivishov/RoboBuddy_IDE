#!/usr/bin/env python3
"""Regenerate committed MicroDuck task models from the vendored pinned source MJCF.

The source import itself is performed by finalize_microduck_full_plant.py. This
compatibility entry point intentionally no longer contains fitted primitive
collision geometry.
"""
from __future__ import annotations

import argparse
from pathlib import Path

from finalize_microduck_full_plant import REPO, SOURCE_DIR, generate_models, patch_registry

TRACKED = [
    REPO / "models/microduck/walk.xml",
    REPO / "models/microduck/walk_lowtraction.xml",
    REPO / "models/microduck/groundcontact.xml",
    REPO / "models/microduck/kick.xml",
    REPO / "models/microduck/SHA256SUMS",
    REPO / "src/physics/microduck-model-package.js",
]


def snapshot():
    return {path: path.read_bytes() if path.exists() else None for path in TRACKED}


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    if not (SOURCE_DIR / "robot_walk.xml").exists() or not (SOURCE_DIR / "robot_allcollisions.xml").exists():
        raise SystemExit("Pinned source MJCF is not vendored yet; run scripts/finalize_microduck_full_plant.py --source-checkout <microduck_rl checkout> first")
    before = snapshot() if args.check else None
    hashes = generate_models()
    patch_registry(hashes)
    if args.check:
        after = snapshot()
        changed = [str(path.relative_to(REPO)) for path in TRACKED if before[path] != after[path]]
        if changed:
            raise SystemExit("Generated MicroDuck source models differ from the committed files: " + ", ".join(changed))
        print("MicroDuck source-model generation is byte-reproducible")
    else:
        for name, digest in sorted(hashes.items()):
            print(f"{name}: {digest}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
