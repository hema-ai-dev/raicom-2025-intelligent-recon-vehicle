#!/usr/bin/env python3
"""Validate the public showcase without importing ROS or ML dependencies."""

import ast
import csv
import hashlib
import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]

EXPECTED_MODELS = {
    "models/soldier/final_training_best.pt": "1C84949A79C15036730BD4A87877BDF1E9A947D9EE87FB33937803DB161ADCD4",
    "models/soldier/robot_deployed_best.pt": "78F8D2E91C832CBFCAA1AD56888F8DC1E939AD94F33AF5A88AF53B9C9C43EC7A",
    "models/target/best.pt": "8FEC08A1ACA0BDA03E6FFC723B574B2CF577565311507A31043870923B99E166",
}

CORE_PYTHON = (
    "vision/soldier/train.py",
    "vision/soldier/infer.py",
    "vision/soldier/evaluate.py",
    "vision/target/train.py",
    "vision/target/infer.py",
    "vision/target/evaluate.py",
    "robot/ros_ws/src/nav_demo/scripts/mission_controller.py",
    "robot/ros_ws/src/nav_demo/scripts/soldier_detector.py",
    "robot/ros_ws/src/nav_demo/scripts/gun_sweep_server.py",
    "robot/prototype/auto_aim_controller.py",
)

REQUIRED = (
    "README.md",
    "README.zh-CN.md",
    "NOTICE.md",
    "docs/ARCHITECTURE.md",
    "docs/ARCHITECTURE.zh-CN.md",
    "docs/DESIGN_DECISIONS.md",
    "docs/DESIGN_DECISIONS.zh-CN.md",
    "models/MODEL_CARD.md",
    "models/MODEL_CARD.zh-CN.md",
    "vision/README.md",
    "vision/README.zh-CN.md",
    "robot/ros_ws/src/nav_demo/map/xiaomingPP.yaml",
    "robot/ros_ws/src/nav_demo/map/xiaomingPP.pgm",
)

TEXT_SUFFIXES = {".py", ".md", ".yaml", ".yml", ".xml", ".launch", ".txt", ".cfg", ".sh"}
FORBIDDEN_PATTERNS = (
    re.compile(r"[A-Za-z]:\\睿抗"),
    re.compile(r"BEGIN [A-Z ]*PRIVATE KEY"),
)


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest().upper()


def validate_required(errors):
    for relative in REQUIRED:
        if not (ROOT / relative).is_file():
            errors.append("missing required file: {}".format(relative))


def validate_python(errors):
    for relative in CORE_PYTHON:
        path = ROOT / relative
        try:
            ast.parse(path.read_text(encoding="utf-8"), filename=relative)
        except Exception as exc:
            errors.append("python parse failed for {}: {}".format(relative, exc))


def validate_xml(errors):
    candidates = list((ROOT / "robot/ros_ws/src").rglob("package.xml"))
    candidates += list((ROOT / "robot/ros_ws/src").rglob("*.launch"))
    for path in candidates:
        try:
            ET.parse(path)
        except Exception as exc:
            errors.append("XML parse failed for {}: {}".format(path.relative_to(ROOT), exc))


def validate_map(errors):
    yaml_path = ROOT / "robot/ros_ws/src/nav_demo/map/xiaomingPP.yaml"
    first_line = yaml_path.read_text(encoding="utf-8").splitlines()[0]
    image = first_line.partition(":")[2].strip()
    if Path(image).is_absolute():
        errors.append("map image path must be relative")
    elif not (yaml_path.parent / image).is_file():
        errors.append("map image does not exist: {}".format(image))


def validate_models(errors):
    for relative, expected in EXPECTED_MODELS.items():
        path = ROOT / relative
        if not path.is_file():
            errors.append("missing model: {}".format(relative))
        elif path.stat().st_size < 1024 * 1024:
            errors.append("{} looks like an unresolved Git LFS pointer".format(relative))
        elif sha256(path) != expected:
            errors.append("model hash mismatch: {}".format(relative))


def validate_metrics(errors):
    expected = {
        "vision/soldier/training_record/results.csv": ("0.97537", "0.74299"),
        "vision/target/training_record/results.csv": ("0.993", "0.74579"),
    }
    for relative, (map50, map5095) in expected.items():
        with (ROOT / relative).open(encoding="utf-8", newline="") as handle:
            rows = list(csv.DictReader(handle))
        if not rows:
            errors.append("empty training record: {}".format(relative))
            continue
        row = {key.strip(): value.strip() for key, value in rows[-1].items()}
        if row.get("metrics/mAP50(B)") != map50 or row.get("metrics/mAP50-95(B)") != map5095:
            errors.append("unexpected final metrics: {}".format(relative))


def validate_public_boundary(errors):
    forbidden_names = {"private_reference", "captured_images", "full_data", "GSdata"}
    for path in ROOT.rglob("*"):
        if any(part in forbidden_names for part in path.parts):
            errors.append("private or raw-data path present: {}".format(path.relative_to(ROOT)))
        if path == Path(__file__).resolve():
            continue
        if not path.is_file() or path.suffix.lower() not in TEXT_SUFFIXES:
            continue
        text = path.read_text(encoding="utf-8", errors="replace")
        for pattern in FORBIDDEN_PATTERNS:
            if pattern.search(text):
                errors.append("forbidden path or secret marker in {}".format(path.relative_to(ROOT)))


def main() -> int:
    errors = []
    validate_required(errors)
    validate_python(errors)
    validate_xml(errors)
    validate_map(errors)
    validate_models(errors)
    validate_metrics(errors)
    validate_public_boundary(errors)

    if errors:
        print("Repository validation failed:")
        for error in errors:
            print("- {}".format(error))
        return 1

    print("Repository validation passed.")
    print("Checked core Python, ROS XML, map assets, model hashes, metrics and public boundary.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
