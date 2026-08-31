"""Recompute the committed MicroDuck fixture with isolated ONNX Runtime CPU inference."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path

import numpy as np
import onnxruntime as ort


ROOT = Path(__file__).resolve().parents[1] / "assets" / "microduck"
FIXTURE = ROOT / "fixtures" / "inference-parity.json"


def main() -> None:
    fixture = json.loads(FIXTURE.read_text(encoding="utf-8"))
    input_tensor = np.asarray(fixture["input"], dtype=np.float32).reshape(1, 61)
    tolerance = float(fixture["tolerance"])
    for name, item in fixture["policies"].items():
        model_path = ROOT / item["path"]
        digest = hashlib.sha256(model_path.read_bytes()).hexdigest()
        if digest != item["sha256"]:
            raise SystemExit(f"{name}: model SHA256 does not match the fixture")
        session = ort.InferenceSession(str(model_path), providers=["CPUExecutionProvider"])
        actual = session.run(None, {item["inputName"]: input_tensor})[0].reshape(-1)
        expected = np.asarray(item["output"], dtype=np.float32)
        maximum_error = float(np.max(np.abs(actual - expected)))
        if list(actual.shape) != [14] or maximum_error > tolerance:
            raise SystemExit(f"{name}: shape={list(actual.shape)} max_error={maximum_error}")
        print(f"{name}: 1x61 -> 1x14 CPU fixture verified; max_error={maximum_error}")


if __name__ == "__main__":
    main()
