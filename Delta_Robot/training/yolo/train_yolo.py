"""
train_yolo.py -- train the one-class grayscale strawberry detector, then export it for the Pi.

    .venv/bin/python yolo/train_yolo.py DATA_YAML [--epochs 100] [--model yolo11n.pt] [--imgsz 640]

Outputs in training/yolo/runs/<name>/:  weights/best.pt  (+ best_ncnn_model/ for the Pi CPU)  and the usual Ultralytics plots.
Colour augmentation is switched off (the pictures are gray); brightness, blur-like scale changes and flips stay on.
Copy to the Pi as described in docs/YOLO_TRAINING.md (step 4). Weights are never committed (*.pt is in .gitignore).
"""
import argparse
import os

HERE = os.path.dirname(os.path.abspath(__file__))


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("data")
    ap.add_argument("--model", default="yolo11n.pt")
    ap.add_argument("--epochs", type=int, default=100)
    ap.add_argument("--imgsz", type=int, default=640)
    ap.add_argument("--batch", type=int, default=32)
    ap.add_argument("--name", default="strawberry_gray")
    ap.add_argument("--no-export", action="store_true")
    a = ap.parse_args()

    from ultralytics import YOLO
    model = YOLO(a.model)
    model.train(data=a.data, epochs=a.epochs, imgsz=a.imgsz, batch=a.batch, project=os.path.join(HERE, "runs"), name=a.name,
                exist_ok=True, patience=25, seed=0, workers=8,
                hsv_h=0.0, hsv_s=0.0, hsv_v=0.4,          # gray pictures: no hue/saturation changes, keep brightness changes
                fliplr=0.5, flipud=0.0, degrees=5.0, scale=0.5, mosaic=1.0, close_mosaic=10)
    best = os.path.join(HERE, "runs", a.name, "weights", "best.pt")
    m = YOLO(best)
    metrics = m.val(data=a.data, split="test", imgsz=a.imgsz, project=os.path.join(HERE, "runs"), name=a.name + "_test",
                    exist_ok=True)
    print(f"TEST  mAP50={metrics.box.map50:.3f}  mAP50-95={metrics.box.map:.3f}  "
          f"precision={metrics.box.mp:.3f}  recall={metrics.box.mr:.3f}")
    if not a.no_export:
        path = m.export(format="ncnn", imgsz=a.imgsz)
        print("NCNN model for the Pi:", path)
    print("PyTorch weights:", best)


if __name__ == "__main__":
    main()
