from pathlib import Path

from ultralytics import YOLO


HERE = Path(__file__).resolve().parent


def main() -> None:
    model = YOLO("yolo11n.pt")
    model.train(
        data=str(HERE / "data.example.yaml"),
        epochs=200,
        imgsz=640,
        batch=8,
        workers=4,
        patience=50,
        optimizer="SGD",
        close_mosaic=10,
        degrees=5.0,
        scale=0.5,
        hsv_h=0.015,
        hsv_s=0.7,
        hsv_v=0.4,
        project=str(HERE / "runs"),
        name="yolov11_custom",
        exist_ok=True,
    )


if __name__ == "__main__":
    main()
