import argparse
from pathlib import Path

from ultralytics import YOLO


PROJECT_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_MODEL = PROJECT_ROOT / "models" / "soldier" / "final_training_best.pt"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run 2025 soldier recognition inference")
    parser.add_argument("source", help="Image, directory, video, camera index, or stream URL")
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL)
    parser.add_argument("--confidence", type=float, default=0.25)
    parser.add_argument("--output", type=Path, default=Path(__file__).resolve().parent / "runs")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    model = YOLO(str(args.model))
    model.predict(
        source=args.source,
        imgsz=640,
        conf=args.confidence,
        save=True,
        project=str(args.output),
        name="predict",
        exist_ok=True,
    )


if __name__ == "__main__":
    main()
