# 水弹枪靶标识别

该模型用于深度相机闭环瞄准原型，不代表最终赛场策略。最终方案采用固定范围扫射。

```bash
python vision/target/infer.py image.jpg
python vision/target/evaluate.py --data path/to/data.yaml
python vision/target/train.py
```

训练前必须编辑 `data.example.yaml`。原始靶标数据没有随仓库发布。
