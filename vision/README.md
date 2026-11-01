# Vision Workflows

English | [简体中文](README.zh-CN.md)

This directory contains portable training, inference, and evaluation entry points reconstructed from the historical vision code. The original datasets are not published, so training and evaluation require local data paths in the corresponding `data.example.yaml` files.

## Soldier-board recognition

The soldier workflow recognizes enemy, friendly, and hostage boards. Inference and evaluation default to `models/soldier/final_training_best.pt`.

```bash
python vision/soldier/infer.py image.jpg
python vision/soldier/evaluate.py --data path/to/data.yaml
python vision/soldier/train.py
```

Before training, edit `vision/soldier/data.example.yaml` and replace its placeholder paths with local dataset paths. The retained training record is under `vision/soldier/training_record/`. Its final row is epoch 125: the run requested 250 epochs but stopped early.

## Water-gun target detection

The target workflow defaults to `models/target/best.pt` and belongs to the experimental depth-camera target-detection work. It does not represent the final competition strategy, which used a deterministic sweep.

```bash
python vision/target/infer.py image.jpg
python vision/target/evaluate.py --data path/to/data.yaml
python vision/target/train.py
```

Before training, edit `vision/target/data.example.yaml` and provide local dataset paths. The original target dataset is not published. Compatibility between the retained YOLOv5-style closed-loop controller and the packaged YOLO11 target weight has not been revalidated.

For model provenance, hashes, metrics, and limitations, see the [model card](../models/MODEL_CARD.md).
