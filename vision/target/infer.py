import argparse
from pathlib import Path

from ultralytics import YOLO


PROJECT_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_MODEL = PROJECT_ROOT / "models" / "target" / "best.pt"


def main() -> None:
    parser = argparse.ArgumentParser(description="Run water-gun target detection")
    parser.add_argument("source", help="Image, directory, video, camera index, or stream URL")
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL)
    parser.add_argument("--confidence", type=float, default=0.25)
    parser.add_argument("--output", type=Path, default=Path(__file__).resolve().parent / "runs")
    args = parser.parse_args()

    YOLO(str(args.model)).predict(
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
