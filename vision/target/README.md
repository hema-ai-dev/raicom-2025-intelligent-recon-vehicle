# Water-Gun Target Detection

English | [简体中文](README.zh-CN.md)

This model supports the depth-camera closed-loop aiming prototype. It does not represent the final competition strategy, which used a fixed-range sweep.

```bash
python vision/target/infer.py image.jpg
python vision/target/evaluate.py --data path/to/data.yaml
python vision/target/train.py
```

Edit `data.example.yaml` before training. The original target dataset is not published.
