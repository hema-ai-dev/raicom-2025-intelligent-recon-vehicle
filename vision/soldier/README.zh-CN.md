# 兵人识别

[English](README.md) | 简体中文

`train.py`、`infer.py` 和 `evaluate.py` 是从历史训练脚本整理出的可移植入口。默认推理权重是 `models/soldier/final_training_best.pt`。

```bash
python vision/soldier/infer.py image.jpg
python vision/soldier/evaluate.py --data path/to/data.yaml
python vision/soldier/train.py
```

训练前必须编辑 `data.example.yaml`，将三个路径改为本机数据目录。数据没有随仓库发布。

训练记录位于 `training_record/`。最后一行对应 epoch 125；训练参数最初请求 250 epochs，但由早停机制提前结束。
