# Soldier-Board Recognition

English | [简体中文](README.zh-CN.md)

`train.py`, `infer.py`, and `evaluate.py` are portable entry points reconstructed from the historical training scripts. Inference defaults to `models/soldier/final_training_best.pt`.

```bash
python vision/soldier/infer.py image.jpg
python vision/soldier/evaluate.py --data path/to/data.yaml
python vision/soldier/train.py
```

Before training, edit `data.example.yaml` and replace its three placeholder paths with local dataset paths. The dataset is not published.

The retained training record is under `training_record/`. Its final row is epoch 125: the run requested 250 epochs but stopped early.
