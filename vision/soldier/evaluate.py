import argparse
from pathlib import Path

from ultralytics import YOLO


PROJECT_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_MODEL = PROJECT_ROOT / "models" / "soldier" / "final_training_best.pt"


def main() -> None:
    parser = argparse.ArgumentParser(description="Evaluate the 2025 soldier recognition model")
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL)
    parser.add_argument("--data", type=Path, default=Path(__file__).resolve().parent / "data.example.yaml")
    parser.add_argument("--split", choices=("val", "test"), default="val")
    args = parser.parse_args()

    result = YOLO(str(args.model)).val(data=str(args.data), split=args.split)
    print(f"mAP50: {result.box.map50:.3f}")
    print(f"mAP50-95: {result.box.map:.3f}")
    print(f"Precision: {result.box.mp:.3f}")
    print(f"Recall: {result.box.mr:.3f}")


if __name__ == "__main__":
    main()
