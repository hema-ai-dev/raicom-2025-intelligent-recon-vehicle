# 视觉工作流

[English](README.md) | 简体中文

本目录包含从历史视觉代码中整理出的训练、推理和评估入口。原始数据集没有公开，因此训练与评估前需要在对应的 `data.example.yaml` 中填写本地数据路径。

## 兵人识别

兵人工作流用于识别敌方、友方和人质兵人牌。推理与评估默认使用 `models/soldier/final_training_best.pt`。

```bash
python vision/soldier/infer.py image.jpg
python vision/soldier/evaluate.py --data path/to/data.yaml
python vision/soldier/train.py
```

训练前请编辑 `vision/soldier/data.example.yaml`，将占位路径替换为本地数据路径。保留的训练记录位于 `vision/soldier/training_record/`；最后一行对应 epoch 125，训练参数原先请求 250 epochs，但由早停机制提前结束。

## 水弹枪靶标识别

靶标工作流默认使用 `models/target/best.pt`，属于深度相机靶标检测的实验性研发内容，不代表最终赛场策略；最终方案采用行为更确定的固定范围扫射。

```bash
python vision/target/infer.py image.jpg
python vision/target/evaluate.py --data path/to/data.yaml
python vision/target/train.py
```

训练前请编辑 `vision/target/data.example.yaml` 并填写本地数据路径。原始靶标数据集没有公开。现存 YOLOv5 风格闭环控制器与仓库所含 YOLO11 靶标权重之间的兼容性尚未重新验证。

模型来源、哈希、指标和限制见[模型卡](../models/MODEL_CARD.zh-CN.md)。
