# Reproducibility Notes

English | [简体中文](REPRODUCIBILITY.zh-CN.md)

## What has been verified

- The public core Python files pass syntax parsing.
- Catkin manifests and launch files parse as XML.
- The occupancy-map YAML refers to an in-repository image with a relative path.
- All three model SHA-256 hashes match their archived source files.
- The published metrics can be recomputed from the retained `results.csv` files.

## Vision environment

Python 3.10 or newer is recommended for inspecting the models:

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
python vision/soldier/infer.py image.jpg
```

Training and evaluation require editing the relevant `data.example.yaml`. The raw datasets are not included, so the exact historical split cannot be reconstructed from this repository alone.

## ROS environment

The vehicle targeted ROS Melodic. At minimum, a reproduction environment needs:

- `navigation`, `gmapping`, `map_server`, and `amcl`;
- `robot_localization` and `laser_scan_matcher`;
- YDLIDAR and WIT Motion IMU drivers that match the actual devices;
- Python 3 packages including `ultralytics`, `opencv-python`, and `pyserial`.

```bash
cd robot/ros_ws
catkin_make
source devel/setup.bash
```

A successful build does not mean the robot is safe to drive. Verify `/dev/gun`, the chassis serial port, LiDAR and IMU device names, camera indices, the TF tree, wheel-scale parameters, and the map origin before enabling hardware.

## Model files

The weights are managed by Git LFS. If a cloned `.pt` file is only a small pointer file, install Git LFS and fetch the objects:

```bash
git lfs install
git lfs pull
```

## Repository validation

`tools/validate_repo.py` requires neither ROS nor Ultralytics. It checks core syntax, ROS XML, map references, model hashes, retained metrics, and the public/private boundary:

```bash
python tools/validate_repo.py
```
