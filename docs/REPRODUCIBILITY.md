# 复现说明

## 已验证的内容

- 公开版核心 Python 文件可以通过语法解析。
- catkin 包清单和 launch 文件可以作为 XML 解析。
- 地图 YAML 使用仓库内相对图片路径。
- 三份模型权重与归档源文件 SHA-256 一致。
- 训练指标可以从保留的 `results.csv` 重算。

## 视觉环境

建议使用 Python 3.10 或更新版本单独查看模型：

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
python vision/soldier/infer.py image.jpg
```

训练和评估需要修改相应 `data.example.yaml`。仓库没有附带原始数据集，也无法仅凭现有文件重现完全相同的数据划分。

## ROS 环境

原车使用 ROS Melodic。至少需要安装：

- `navigation`、`gmapping`、`map_server`、`amcl`；
- `robot_localization`、`laser_scan_matcher`；
- 与实际硬件匹配的 YDLIDAR 和 WIT Motion IMU 驱动；
- Python 3 环境中的 `ultralytics`、`opencv-python` 与 `pyserial`。

```bash
cd robot/ros_ws
catkin_make
source devel/setup.bash
```

编译成功仍不等于可以直接驱动车辆。启动前必须核对 `/dev/gun`、底盘串口、雷达与 IMU 设备名、摄像头编号、TF 树、轮速比例和地图原点。

## 模型文件

模型由 Git LFS 管理。克隆后若看到只有几百字节的 `.pt` 指针文件，请先安装 Git LFS，再执行：

```bash
git lfs install
git lfs pull
```

## 仓库自检

`tools/validate_repo.py` 不依赖 ROS 或 Ultralytics，可检查核心语法、XML、地图引用、权重哈希、训练指标和敏感路径：

```bash
python tools/validate_repo.py
```
