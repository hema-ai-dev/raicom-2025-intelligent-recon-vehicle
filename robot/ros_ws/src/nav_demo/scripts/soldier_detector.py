#!/usr/bin/env python3
"""Capture one frame and summarize soldier-board detections."""

import argparse
import time
from pathlib import Path
from typing import Dict

import cv2
from ultralytics import YOLO


PROJECT_ROOT = Path(__file__).resolve().parents[5]
DEFAULT_MODEL = PROJECT_ROOT / "models" / "soldier" / "robot_deployed_best.pt"
DEFAULT_OUTPUT = PROJECT_ROOT / "artifacts" / "camera_detections"

ROLE_MAP = {
    "red": "enemy",
    "blue": "enemy",
    "white": "hostage",
    "colorful": "friend",
}
ROLES = ("enemy", "friend", "hostage")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Capture and classify one competition region")
    parser.add_argument("--region", required=True, help="Region identifier, for example 1 or A")
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL)
    parser.add_argument("--save-dir", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--camera", type=int, default=2)
    parser.add_argument("--confidence", type=float, default=0.25)
    parser.add_argument("--warmup-frames", type=int, default=8)
    return parser.parse_args()


def capture_frame(camera: int, warmup_frames: int):
    cap = cv2.VideoCapture(camera)
    if not cap.isOpened():
        raise RuntimeError("cannot open camera {}".format(camera))

    try:
        for _ in range(max(1, warmup_frames)):
            ok, _ = cap.read()
            if not ok:
                time.sleep(0.05)
        ok, frame = cap.read()
    finally:
        cap.release()

    if not ok:
        raise RuntimeError("cannot capture a frame from camera {}".format(camera))
    return frame


def count_roles(result, model) -> Dict[str, int]:
    counts = {role: 0 for role in ROLES}
    names = getattr(result, "names", None) or getattr(model, "names", {})
    for box in result.boxes or []:
        class_id = int(box.cls[0])
        source_name = str(names.get(class_id, "")).strip().lower()
        role = ROLE_MAP.get(source_name)
        if role:
            counts[role] += 1
    return counts


def main() -> int:
    args = parse_args()
    if not args.model.is_file():
        raise FileNotFoundError("model not found: {}".format(args.model))
    if not 0.0 <= args.confidence <= 1.0:
        raise ValueError("--confidence must be between 0 and 1")

    args.save_dir.mkdir(parents=True, exist_ok=True)
    model = YOLO(str(args.model))
    frame = capture_frame(args.camera, args.warmup_frames)
    result = model(frame, conf=args.confidence, verbose=False)[0]
    counts = count_roles(result, model)

    timestamp = time.strftime("%Y%m%d_%H%M%S")
    output = args.save_dir / "region{}_{}.jpg".format(args.region, timestamp)
    if not cv2.imwrite(str(output), result.plot()):
        raise RuntimeError("failed to write result: {}".format(output))

    print(
        "region {}: enemy={} hostage={} friend={} output={}".format(
            args.region,
            counts["enemy"],
            counts["hostage"],
            counts["friend"],
            output,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
