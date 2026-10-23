#!/usr/bin/env python3
import argparse
import os
import sys
import time
import math
import re
import glob  # 新增导入
from pathlib import Path
from collections import deque

import pathlib
pathlib.WindowsPath = pathlib.PosixPath


import numpy as np
import torch
import serial
import cv2
import rospy
from std_srvs.srv import Trigger, TriggerResponse
from geometry_msgs.msg import Twist
from std_msgs.msg import String  # 导入String消息类型

# 新增YOLO相关导入
from models.common import DetectMultiBackend
from utils.dataloaders import LoadStreams
from utils.general import (check_img_size, non_max_suppression, scale_boxes)
from utils.plots import Annotator


# 新增：定义increment_path函数（解决之前的NameError）
def increment_path(path, exist_ok=False, sep='', mkdir=False):
    path = Path(path)
    if path.exists() and not exist_ok:
        suffix = path.suffix
        path = path.with_suffix('')
        dirs = glob.glob(f"{path}{sep}*")
        matches = [re.search(rf"{path.stem}{sep}(\d+)", str(d)) for d in dirs]
        i = [int(m.groups()[0]) for m in matches if m]
        n = max(i) + 1 if i else 2
        path = Path(f"{path}{sep}{n}{suffix}")
    dir = path if path.suffix == '' else path.parent
    if mkdir:
        dir.mkdir(parents=True, exist_ok=True)
    return path


# 相机内参 - 根据实际相机校准
CAMERA_MATRIX = np.array([
    [600, 0, 320],
    [0, 600, 240],
    [0, 0, 1]
], dtype=np.float32)

DIST_COEFFS = np.array([0.1, 0.05, 0, 0, 0], dtype=np.float32)

# 目标实际尺寸 (cm)
TARGET_WIDTH_CM = 5.7
TARGET_HEIGHT_CM = 7.5

# 测距校准参数
DISTANCE_CALIBRATION_FACTOR = 0.85

# 枪口相对于相机的位置 (cm)
GUN_OFFSET_X = -5.0  # 枪口在相机左边5cm
GUN_OFFSET_Z = 15.0  # 枪口在相机前方15cm

# 舵机参数
SERVO_HORIZONTAL = "horizontal"
SERVO_VERTICAL = "vertical"
HORIZONTAL_MIN = 0
HORIZONTAL_MAX = 180
VERTICAL_MIN = 70
VERTICAL_MAX = 110

# 设备配置
DEVICE = torch.device("cuda" if torch.cuda.is_available() else "cpu")

# 瞄准判定阈值（像素距离）
AIM_PIXEL_THRESHOLD = 7.0  # 靶点距离目标落点≤7像素即判定为瞄准（增加容错）


class GunControllerNode:
    def __init__(self, opt):
        self.opt = opt
        self.compensation_data = []  # 存储补偿数据
        self.load_compensation_data()  # 加载补偿数据
        self.calculate_data_bounds()   # 计算数据的最大最小值范围
        self.calculate_center_point()  # 计算落点集中点
        self.init_system_parameters()
        self.init_serial()
        self.init_vision_model()
        self.init_ros_components()  # 初始化ROS组件（包括新增的发布者）
        self.init_control_system()

        # 摄像头状态标记
        self.camera_active = False  # 摄像头是否激活
        self.window_name = "Camera View"  # 统一窗口名称

        print(f"系统初始化完成 - 舵机角度限制：水平{ HORIZONTAL_MIN }-{ HORIZONTAL_MAX }度，垂直{ VERTICAL_MIN }-{ VERTICAL_MAX }度")
        print(f"数据范围限制 - X: {self.x_min}-{self.x_max}, Y: {self.y_min}-{self.y_max}, 距离: {self.distance_min:.1f}-{self.distance_max:.1f}cm")
        print(f"落点集中点: ({self.center_x:.1f}, {self.center_y:.1f}) (基于{len(self.compensation_data)}条数据)")
        print(f"瞄准判定阈值: 靶点距离目标落点≤{AIM_PIXEL_THRESHOLD}像素")
        self.check_serial_permissions()
        print("等待服务调用... (使用 rosservice call /gun \"{}\" 启动打靶流程)")

    def load_compensation_data(self):
        """加载buchang.txt中的补偿数据"""
        try:
            with open("buchang.txt", 'r') as f:
                lines = f.readlines()

                pattern = r'\[([\d.]+),\s*(\d+),\s*(\d+)\]'

                for line_num, line in enumerate(lines, 1):
                    line = line.strip()
                    if not line:
                        continue

                    match = re.match(pattern, line)
                    if match:
                        distance = float(match.group(1))
                        x_coord = int(match.group(2))
                        y_coord = int(match.group(3))

                        self.compensation_data.append({
                            'distance': distance,
                            'x': x_coord,
                            'y': y_coord
                        })
                    else:
                        print(f"警告: buchang.txt 第 {line_num} 行格式不正确，已跳过")

            if self.compensation_data:
                self.compensation_data.sort(key=lambda x: x['distance'])
                print(f"成功加载 {len(self.compensation_data)} 条补偿数据")
            else:
                print("警告: 未从buchang.txt加载到有效补偿数据")

        except FileNotFoundError:
            print("警告: 未找到 buchang.txt 文件，将不使用文件补偿数据")
        except Exception as e:
            print(f"加载补偿数据时出错: {str(e)}")

    def find_nearest_compensation_point(self, distance):
        """找到数据集中与给定距离最接近的点"""
        if not self.compensation_data:
            return None

        # 使用二分查找找到最接近的距离点
        left, right = 0, len(self.compensation_data) - 1
        while left < right:
            mid = (left + right) // 2
            if self.compensation_data[mid]['distance'] < distance:
                left = mid + 1
            else:
                right = mid

        # 检查左右两个点哪个更接近
        candidates = [left]
        if left > 0:
            candidates.append(left - 1)

        nearest = min(candidates, key=lambda i: abs(self.compensation_data[i]['distance'] - distance))
        return self.compensation_data[nearest]

    def calculate_data_bounds(self):
        """计算数据的最大最小值范围（用于限制补偿）"""
        if not self.compensation_data:
            # 如果没有数据，使用默认范围
            self.x_min, self.x_max = 250, 320
            self.y_min, self.y_max = 150, 180
            self.distance_min, self.distance_max = 80, 320
            return

        # 提取所有数据的维度
        x_coords = [item['x'] for item in self.compensation_data]
        y_coords = [item['y'] for item in self.compensation_data]
        distances = [item['distance'] for item in self.compensation_data]

        # 计算最大最小值（严格使用数据范围）
        self.x_min = min(x_coords)
        self.x_max = max(x_coords)
        self.y_min = min(y_coords)
        self.y_max = max(y_coords)
        self.distance_min = min(distances)
        self.distance_max = max(distances)

    def calculate_center_point(self):
        """计算落点集中点（平均值）"""
        if not self.compensation_data:
            self.center_x = (self.x_min + self.x_max) / 2  # 使用范围中点
            self.center_y = (self.y_min + self.y_max) / 2
            # 针对远距离偏差预设补偿斜率
            self.x_slope = -0.02  # 负斜率：距离越远，x补偿越向左
            self.y_slope = -0.03  # 负斜率：距离越远，y补偿越向上
            self.x_intercept = 0
            self.y_intercept = 0
            return

        # 计算所有点的平均值作为集中点
        x_coords = [item['x'] for item in self.compensation_data]
        y_coords = [item['y'] for item in self.compensation_data]
        distances = [item['distance'] for item in self.compensation_data]

        self.center_x = np.mean(x_coords)
        self.center_y = np.mean(y_coords)

        # 拟合距离与偏移的线性关系，特别关注远距离数据
        x_offsets = [x - self.center_x for x in x_coords]
        y_offsets = [y - self.center_y for y in y_coords]

        # 对远距离数据赋予更高权重（解决远距离偏差问题）
        weights = np.array([1.0 + 0.005 * (d - self.distance_min) for d in distances])
        weights = weights / np.sum(weights)  # 归一化权重

        # 使用加权线性拟合
        self.x_slope, self.x_intercept = self.weighted_linear_fit(distances, x_offsets, weights)
        self.y_slope, self.y_intercept = self.weighted_linear_fit(distances, y_offsets, weights)

        # 强化远距离补偿效果（针对向右下方偏的问题）
        # 增加x负向补偿（向左）和y负向补偿（向上）
        self.x_slope = self.x_slope - 0.01  # 额外增加向左的补偿趋势
        self.y_slope = self.y_slope - 0.015  # 额外增加向上的补偿趋势

    def weighted_linear_fit(self, x, y, weights):
        """带权重的线性拟合，对远距离点赋予更高权重"""
        if len(x) < 2:
            return -0.02, 0.0  # 默认负斜率，解决向右下偏问题

        x = np.array(x)
        y = np.array(y)
        w = np.array(weights)

        # 加权线性回归
        sum_w = np.sum(w)
        sum_wx = np.sum(w * x)
        sum_wy = np.sum(w * y)
        sum_wxx = np.sum(w * x * x)
        sum_wxy = np.sum(w * x * y)

        denominator = sum_w * sum_wxx - sum_wx * sum_wx
        if denominator == 0:
            return -0.02, 0.0

        slope = (sum_w * sum_wxy - sum_wx * sum_wy) / denominator
        intercept = (sum_wy * sum_wxx - sum_wx * sum_wxy) / denominator

        return slope, intercept

    # 新增：计算瞄准点（核心缺失方法）
    def calculate_aim_point(self, target_x, target_y, distance):
        """计算瞄准点（考虑距离补偿）"""
        if not distance:  # 距离未知时不做补偿
            return target_x, target_y, target_x, target_y

        # 原始瞄准点（无补偿）
        raw_aim_x = target_x
        raw_aim_y = target_y

        # 基于距离的线性补偿（使用之前计算的斜率和截距）
        x_offset = self.x_slope * distance + self.x_intercept
        y_offset = self.y_slope * distance + self.y_intercept

        # 远距离时增强补偿效果（解决远距离偏差）
        if distance > 500:
            x_offset *= 1.2  # 远距离水平补偿增强20%
            y_offset *= 1.2  # 远距离垂直补偿增强20%

        # 初始补偿后的瞄准点
        aim_x = target_x + x_offset
        aim_y = target_y + y_offset

        # 如果有匹配的补偿点，使用85%数据补偿 + 15%线性补偿的混合策略
        if self.has_dataset_match:
            nearest = self.find_nearest_compensation_point(distance)
            if nearest:
                # 数据集中该距离对应的偏移量
                data_x_offset = nearest['x'] - self.center_x
                data_y_offset = nearest['y'] - self.center_y

                # 混合两种补偿（85%数据集补偿 + 15%线性补偿）
                aim_x = target_x + 0.85 * data_x_offset + 0.15 * x_offset
                aim_y = target_y + 0.85 * data_y_offset + 0.15 * y_offset

        # 限制瞄准点在图像范围内（假设图像尺寸640x480）
        aim_x = np.clip(aim_x, 0, 640)
        aim_y = np.clip(aim_y, 0, 480)

        return aim_x, aim_y, raw_aim_x, raw_aim_y

    # 系统参数初始化
    def init_system_parameters(self):
        self.save_img = not self.opt.nosave
        self.source = str(self.opt.source)
        self.project = self.opt.project
        self.name = self.opt.name
        self.exist_ok = self.opt.exist_ok

        # 状态管理
        self.gun_state = "idle"
        self.fire_start_time = 0
        self.last_adjust_time = 0
        self.last_serial_error = 0
        self.serial_reconnect_delay = 2

        # 串口参数
        self.serial_port = self.opt.port
        self.serial_baudrate = self.opt.baudrate
        self.serial_timeout = 1.5
        self.ser = None

        # 串口状态跟踪
        self.serial_status = "未初始化"
        self.command_failure_count = 0
        self.max_failure_before_reinit = 3

        # 保存目录
        self.save_dir = increment_path(Path(self.opt.project) / self.opt.name, exist_ok=self.opt.exist_ok)
        (self.save_dir / "labels" if self.opt.save_txt else self.save_dir).mkdir(parents=True, exist_ok=True)

    # 视觉模型初始化
    def init_vision_model(self):
        self.device =  torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.model = DetectMultiBackend(
            self.opt.weights,
            device=self.device,
            dnn=self.opt.dnn,
            data=self.opt.data,
            fp16=self.opt.half
        )
        self.stride, self.names, self.pt = self.model.stride, self.model.names, self.model.pt
        self.imgsz = check_img_size(self.opt.imgsz, s=self.stride)

        self.dataset = None  # 摄像头数据流（按需初始化）
        self.vid_path, self.vid_writer = [None], [None]

        # 强制禁用界面显示 - 摄像头在后台静默工作
        self.view_img = False  # self.opt.view_img
        self.windows = []

    # ROS组件初始化 - 新增打靶成功话题发布者
    def init_ros_components(self):
        # 确保ROS节点已初始化
        if not rospy.core.is_initialized():
            rospy.init_node('gun_controller', anonymous=True, log_level=rospy.WARN)

        self.adjust_pub = rospy.Publisher('/gun_adjust', Twist, queue_size=10)
        self.gun_service = rospy.Service('/gun', Trigger, self.gun_callback)
        # 新增：创建打靶成功话题发布者，话题名为/gun_success，消息类型为String
        self.success_pub = rospy.Publisher('/gun_success', String, queue_size=10, latch=True)

    # 控制系统初始化
    def init_control_system(self):
        self.servo_horizontal = 90.0  # 水平初始角度
        self.servo_vertical = 90.0    # 垂直初始角度
        self.target_history = deque(maxlen=3)

        # 距离相关参数
        self.distance_history = deque(maxlen=10)  # 存储前10次距离值
        self.fixed_distance = None  # 固定后的距离值
        self.distance_fixed = False  # 距离是否已固定的标志
        self.has_dataset_match = False  # 标记10次距离中是否有任何一次在数据集中有匹配
        self.distance_update_counter = 0  # 距离更新计数器
        self.DISTANCE_UPDATE_INTERVAL = 50  # 每50帧更新一次距离（约5秒）

        # 控制参数
        self.MIN_ADJUST_INTERVAL = 0.5
        self.ADJUST_SETTLE_TIME = 0.6

        # 相机参数
        self.fx = CAMERA_MATRIX[0, 0]
        self.fy = CAMERA_MATRIX[1, 1]
        self.cx = CAMERA_MATRIX[0, 2]  # 图像中心点x坐标
        self.cy = CAMERA_MATRIX[1, 2]  # 图像中心点y坐标
        self.is_adjusting = False

        # PID控制器（针对远距离调整参数）
        self.pid_h = {'Kp': 0.5, 'Ki': 0.04, 'Kd': 0.1, 'err_sum': 0, 'last_err': 0}
        self.pid_v = {'Kp': 0.6, 'Ki': 0.05, 'Kd': 0.12, 'err_sum': 0, 'last_err': 0}
        self.PID_MAX_INCREMENT = 3.0  # 限制单次最大调整量

        # 自动开火参数
        self.stable_counter = 0  # 目标稳定在落点区域的计数
        self.STABLE_THRESHOLD = 5  # 目标需要稳定的帧数（约0.5秒）
        self.FIRE_DURATION = 1.0  # 单次开火持续时间（秒）
        self.COOLDOWN_DURATION = 2.0  # 两次开火的冷却时间（秒）
        self.last_fire_end_time = 0  # 上次开火结束时间
        self.target_lost_counter = 0  # 目标丢失计数器
        self.TARGET_LOST_THRESHOLD = 3  # 连续多少帧未检测到目标判定为未命中

        # 命中检测相关参数
        self.previous_target_exists = False  # 上一帧是否有目标
        self.target_acquired = False  # 是否已经获取过目标（用于判断是否是新目标）
        self.active = False  # 本次服务调用是否活跃

    # 初始化摄像头（单独提取方法）
    def init_camera(self):
        """初始化摄像头并返回数据流"""
        try:
            self.dataset = LoadStreams(
                self.source,
                img_size=self.imgsz,
                stride=self.stride,
                auto=self.pt,
                vid_stride=1
            )
            self.model.warmup(imgsz=(1 if self.pt else len(self.dataset), 3, *self.imgsz))
            self.camera_active = True

            # 创建显示窗口（只在第一次创建）- 已禁用界面显示
            # cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)

            print("摄像头初始化成功")
            return self.dataset
        except Exception as e:
            print(f"摄像头初始化失败: {str(e)}")
            self.dataset = None
            self.camera_active = False
            return None

    # # 关闭摄像头（单独提取方法）
    # def close_camera(self):
    #     """强制关闭摄像头并释放资源"""
    #     if self.camera_active:
    #         print("强制关闭摄像头...")
    #         self.dataset = None  # 释放数据流（LoadStreams会自动关闭摄像头）
    #         self.camera_active = False
    #         # 销毁所有窗口
    #         if cv2.getWindowProperty(self.window_name, cv2.WND_PROP_VISIBLE) >= 1:
    #             cv2.destroyWindow(self.window_name)
    #         print("摄像头已关闭")
    def close_camera(self):
        """强制关闭摄像头并释放资源"""
        try:
            if self.camera_active:
                print("正在关闭摄像头...")

                # 释放数据集资源
                if hasattr(self, 'dataset') and self.dataset is not None:
                    try:
                        # 尝试显式释放资源
                        if hasattr(self.dataset, 'cap'):
                            self.dataset.cap.release()
                        elif hasattr(self.dataset, 'streams'):
                            for stream in self.dataset.streams:
                                if hasattr(stream, 'cap'):
                                    stream.cap.release()
                    except Exception as e:
                        print(f"释放摄像头资源时出错: {str(e)}")
                    finally:
                        self.dataset = None

                self.camera_active = False

                # 销毁所有窗口
                try:
                    if hasattr(self, 'window_name'):
                        if cv2.getWindowProperty(self.window_name, cv2.WND_PROP_VISIBLE) >= 1:
                            cv2.destroyWindow(self.window_name)
                        cv2.waitKey(1)  # 确保窗口关闭
                except Exception as e:
                    print(f"关闭窗口时出错: {str(e)}")

                print("摄像头和资源已完全关闭")

        except Exception as e:
            print(f"关闭摄像头过程中发生严重错误: {str(e)}")
        finally:
            # 确保状态标记为关闭
            self.camera_active = False

    def check_serial_permissions(self):
        if not os.path.exists(self.serial_port):
            print(f"警告: 串口设备 {self.serial_port} 不存在!")
            return

        try:
            with open(self.serial_port, 'r'):
                pass
        except PermissionError:
            print(f"警告: 没有访问 {self.serial_port} 的权限!")
            print("请运行 'sudo usermod -a -G dialout $USER' 并重启电脑以获取权限")
        except Exception as e:
            print(f"检查串口权限时出错: {str(e)}")

    def init_serial(self):
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()

            self.ser = serial.Serial(
                port=self.serial_port,
                baudrate=self.serial_baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=self.serial_timeout,
                write_timeout=1.0
            )
            time.sleep(1.0)
            if self.ser.is_open:
                self.ser.flushInput()
                self.ser.flushOutput()
                self.serial_status = "已连接"
                self.command_failure_count = 0
                print(f"串口 {self.serial_port} 连接成功")
                return True
            self.serial_status = "打开失败"
            return False
        except Exception as e:
            self.serial_status = f"初始化失败: {str(e)}"
            print(f"串口初始化失败: {str(e)}")
            self.ser = None
            return False

    def check_serial_connection(self):
        if self.command_failure_count >= self.max_failure_before_reinit:
            print(f"连续 {self.max_failure_before_reinit} 次命令失败，尝试重连串口...")
            return self.init_serial()

        if not self.ser or not self.ser.is_open:
            current_time = time.time()
            if current_time - self.last_serial_error > self.serial_reconnect_delay:
                print(f"重连串口 {self.serial_port}...")
                self.last_serial_error = current_time
                return self.init_serial()
            return False
        return True

    def move_servo(self, angle, direction):
        if direction == SERVO_HORIZONTAL:
            angle = np.clip(int(round(angle)), HORIZONTAL_MIN, HORIZONTAL_MAX)
        else:
            angle = np.clip(int(round(angle)), VERTICAL_MIN, VERTICAL_MAX)

        dir_name = "水平" if direction == SERVO_HORIZONTAL else "垂直"
        print(f"\n[舵机控制] 准备{dir_name}舵机移动到 {angle} 度")

        try:
            if not self.check_serial_connection():
                print(f"{dir_name}舵机指令发送失败：串口未连接")
                self.command_failure_count += 1
                return False

            servo_id = 0x01 if direction == SERVO_HORIZONTAL else 0x02
            command = bytearray([0xAA, 0xBB, servo_id, angle, 0x0D, 0x0A])

            self.ser.write(command)
            print(f"指令已发送: {', '.join(f'0x{byte:02X}' for byte in command)}")

            time.sleep(0.1)
            response = self.ser.read(self.ser.in_waiting)
            if response:
                print(f"收到响应: {', '.join(f'0x{byte:02X}' for byte in response)}")

            self.command_failure_count = 0
            return True

        except Exception as e:
            print(f"{dir_name}舵机指令发送失败: {str(e)}")
            self.command_failure_count += 1
            return False

    def send_gun_command(self, is_fire):
        if not self.check_serial_connection():
            return False

        try:
            command = bytearray([0xAA, 0xBB, 0x03, 0x01 if is_fire else 0x00, 0x0D, 0x0A])
            self.ser.write(command)
            print(f"发送{'发射' if is_fire else '停止发射'}指令")
            return True
        except Exception as e:
            print(f"发射指令发送失败: {str(e)}")
            return False

    def calculate_distance(self, target_width_px, target_height_px):
        # 如果距离已经固定，检查是否需要动态更新
        if self.distance_fixed and self.fixed_distance is not None:
            self.distance_update_counter += 1

            # 每50帧更新一次距离，避免距离固定过于严格
            if self.distance_update_counter >= self.DISTANCE_UPDATE_INTERVAL:
                print("距离固定时间较长，允许动态更新")
                self.distance_fixed = False
                self.distance_history.clear()
                self.distance_update_counter = 0
            else:
                return self.fixed_distance

        if target_width_px <= 0 or target_height_px <= 0:
            return None

        distance_by_width = (TARGET_WIDTH_CM * self.fx) / target_width_px
        distance_by_height = (TARGET_HEIGHT_CM * self.fy) / target_height_px

        distance = (distance_by_width + distance_by_height) / 2.0
        distance *= DISTANCE_CALIBRATION_FACTOR

        # 限制距离在数据范围内
        if 50 < distance < 2000:
            self.distance_history.append(distance)

            # 检查当前距离是否在数据集中有匹配点（5cm以内）
            nearest = self.find_nearest_compensation_point(distance)
            if nearest and abs(nearest['distance'] - distance) <= 5:
                self.has_dataset_match = True
                print(f"检测到距离 {distance:.1f}cm 在数据集中有匹配点")

            # 检查是否已收集到10次距离值
            if len(self.distance_history) == 10 and not self.distance_fixed:
                # 计算前10次的加权平均值并固定
                self.fixed_distance = self.calculate_weighted_distance()
                self.distance_fixed = True

                # 输出10次测量中是否有数据集匹配
                if self.has_dataset_match:
                    print(f"\n===== 已收集10次距离值，其中存在数据集匹配点 =====")
                    print(f"===== 固定距离为: {self.fixed_distance:.1f}cm，将使用85%+15%混合补偿 =====")
                else:
                    print(f"\n===== 已收集10次距离值，其中没有数据集匹配点 =====")
                    print(f"===== 固定距离为: {self.fixed_distance:.1f}cm，将使用原始补偿方法 =====")

                return self.fixed_distance
            elif not self.distance_fixed:
                # 尚未收集到10次，返回当前加权平均值
                weighted_distance = self.calculate_weighted_distance()
                return np.clip(weighted_distance, self.distance_min, self.distance_max)

        return None

    def calculate_weighted_distance(self):
        """计算前10次距离值的加权平均，最新的距离权重更高"""
        if not self.distance_history:
            return None

        n = len(self.distance_history)
        # 创建权重数组，最新的距离权重最高
        weights = np.arange(1, n+1)  # [1, 2, ..., n]
        weights = weights / np.sum(weights)  # 归一化权重

        # 计算加权平均
        distances = np.array(self.distance_history)
        weighted_avg = np.sum(distances * weights)

        # 打印权重分配信息
        if len(self.distance_history) < 10:
            print(f"已收集 {len(self.distance_history)}/10 次距离值，当前加权平均: {weighted_avg:.1f}cm")
            if self.has_dataset_match:
                print(f"注意: 已检测到数据集匹配点，最终将使用85%+15%混合补偿")
        else:
            print(f"使用前10次距离加权平均: {weighted_avg:.1f}cm")

        return weighted_avg

    def undistort_point(self, x, y):
        pt = np.array([[[x, y]]], dtype=np.float32)
        undistorted_pt = cv2.undistortPoints(pt, CAMERA_MATRIX, DIST_COEFFS, P=CAMERA_MATRIX)
        return undistorted_pt[0, 0, 0], undistorted_pt[0, 0, 1]

    def calculate_angle_error(self, target_x, target_y, distance):
        aim_x, aim_y, raw_aim_x, raw_aim_y = self.calculate_aim_point(target_x, target_y, distance)

        undistorted_x, undistorted_y = self.undistort_point(target_x, target_y)

        dx = undistorted_x - aim_x
        dy = undistorted_y - aim_y

        angle_err_h = math.atan2(dx, self.fx) * (180 / math.pi)
        angle_err_v = math.atan2(dy, self.fy) * (180 / math.pi) * 1.1

        # 增强远距离误差调整
        if distance and distance > 500:
            angle_err_v *= 1.3  # 垂直方向调整更灵敏
            angle_err_h *= 1.2  # 水平方向调整更灵敏

        return angle_err_h, angle_err_v, aim_x, aim_y, raw_aim_x, raw_aim_y

    def detect_and_process_target(self, im, im0):
        pred = self.model(im, augment=self.opt.augment)
        pred = non_max_suppression(
            pred,
            self.opt.conf_thres,
            self.opt.iou_thres,
            self.opt.classes,
            self.opt.agnostic_nms,
            max_det=self.opt.max_det
        )

        frame_height, frame_width = im0.shape[:2]
        annotator = Annotator(im0, line_width=self.opt.line_thickness, example=str(self.names))
        target_info = None
        aim_point = (self.cx, self.cy)
        raw_aim_point = (self.cx, self.cy)

        # 绘制辅助十字线（贯穿整个图像）
        self.draw_crosshair(im0, frame_width, frame_height)

        for det in pred:
            if det is not None and len(det):
                det_np = det[:, :4].cpu().numpy() if det.is_cuda else det[:, :4].numpy()
                det_np = scale_boxes(im.shape[2:], det_np, im0.shape).round()


                best_det = None
                best_score = 0
                for idx in range(len(det_np)):
                    xyxy = det_np[idx]
                    conf = det[idx, 4].item()
                    area = (xyxy[2] - xyxy[0]) * (xyxy[3] - xyxy[1])
                    score = conf * 0.7 + (1.0 / (1.0 + abs(area - 2000))) * 0.3

                    if score > best_score and conf > self.opt.conf_thres:
                        best_score = score
                        best_det = (xyxy, conf)

                if best_det is not None:
                    xyxy, conf = best_det
                    x1, y1, x2, y2 = map(int, xyxy)

                    target_width = x2 - x1
                    target_height = y2 - y1

                    distance = self.calculate_distance(target_width, target_height)

                    target_x = (x1 + x2) / 2.0
                    target_y = (y1 + y2) / 2.0

                    self.target_history.append((target_x, target_y))
                    avg_x = np.mean([p[0] for p in self.target_history])
                    avg_y = np.mean([p[1] for p in self.target_history])

                    angle_err_h, angle_err_v, aim_x, aim_y, raw_aim_x, raw_aim_y = self.calculate_angle_error(avg_x, avg_y, distance)
                    aim_point = (aim_x, aim_y)
                    raw_aim_point = (raw_aim_x, raw_aim_y)

                    # 计算靶点（目标中心）到目标落点（aim_point）的像素距离
                    pixel_distance = math.hypot(avg_x - aim_x, avg_y - aim_y)
                    in_center = pixel_distance <= AIM_PIXEL_THRESHOLD if distance else False

                    self.draw_target_info(im0, avg_x, avg_y, distance,
                                         aim_point, raw_aim_point, frame_width, frame_height, pixel_distance)

                    target_info = {
                        'x': avg_x,
                        'y': avg_y,
                        'width': target_width,
                        'height': target_height,
                        'distance': distance,
                        'angle_err_h': angle_err_h,
                        'angle_err_v': angle_err_v,
                        'aim_point': aim_point,
                        'raw_aim_point': raw_aim_point,
                        'in_center': in_center,
                        'pixel_distance': pixel_distance  # 新增：记录像素距离
                    }

        # 显示当前中心点坐标
        cv2.putText(im0, f"中心: ({int(self.cx)}, {int(self.cy)})",
                   (int(self.cx) + 10, int(self.cy) - 10),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        # 显示距离状态和补偿方式
        if self.distance_fixed:
            compensation_method = "85%+15%混合补偿" if self.has_dataset_match else "原始补偿方法"
            cv2.putText(im0, f"距离已固定: {self.fixed_distance:.1f}cm",
                       (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            cv2.putText(im0, f"补偿方式: {compensation_method}",
                       (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        else:
            status_text = f"收集距离: {len(self.distance_history)}/10"
            if self.has_dataset_match:
                status_text += " (已检测到数据集匹配)"
            cv2.putText(im0, status_text,
                       (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 165, 255), 2)

        # 显示开火状态
        current_time = time.time()
        if self.gun_state == "firing":
            fire_time_left = self.FIRE_DURATION - (current_time - self.fire_start_time)
            cv2.putText(im0, f"开火中: {fire_time_left:.1f}s",
                       (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        elif self.gun_state == "cooling":
            cool_time_left = self.COOLDOWN_DURATION - (current_time - self.last_fire_end_time)
            cv2.putText(im0, f"冷却中: {cool_time_left:.1f}s",
                       (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
        elif self.gun_state == "ready":
            cv2.putText(im0, "准备就绪，等待目标稳定",
                       (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        elif self.gun_state == "hit_detected":
            cv2.putText(im0, "命中目标! 等待下一次指令...",
                       (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

        # 显示目标丢失状态
        if self.gun_state == "firing" and self.target_lost_counter > 0:
            cv2.putText(im0, f"目标丢失计数: {self.target_lost_counter}/{self.TARGET_LOST_THRESHOLD}",
                       (10, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

        return im0, target_info

    def draw_crosshair(self, im0, frame_width, frame_height):
        """绘制辅助十字线，贯穿整个图像"""
        # 水平线（绿色，线宽2）
        cv2.line(im0, (0, int(self.cy)), (frame_width, int(self.cy)), (0, 255, 0), 2)
        # 垂直线（绿色，线宽2）
        cv2.line(im0, (int(self.cx), 0), (int(self.cx), frame_height), (0, 255, 0), 2)

        # 十字线端点标记
        cross_size = 10  # 端点大小
        cv2.line(im0, (int(self.cx) - cross_size, int(self.cy)), (int(self.cx) + cross_size, int(self.cy)), (0, 255, 0), 3)
        cv2.line(im0, (int(self.cx), int(self.cy) - cross_size), (int(self.cx), int(self.cy) + cross_size), (0, 255, 0), 3)

    def draw_target_info(self, im0, target_x, target_y, distance, aim_point, raw_aim_point, frame_w, frame_h, pixel_distance):
        # 目标中心点
        cv2.circle(im0, (int(target_x), int(target_y)), 5, (0, 0, 255), -1)

        # 补偿之前的落点（原始瞄准点）
        raw_aim_x, raw_aim_y = raw_aim_point
        cv2.circle(im0, (int(raw_aim_x), int(raw_aim_y)), 8, (0, 165, 255), 2)  # 橙色
        cv2.putText(im0, f"补偿前: ({int(raw_aim_x)}, {int(raw_aim_y)})",
                   (int(raw_aim_x) + 10, int(raw_aim_y) + 20),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)

        # 补偿之后的落点（最终瞄准点）
        aim_x, aim_y = aim_point
        cv2.circle(im0, (int(aim_x), int(aim_y)), 8, (255, 0, 0), 2)  # 蓝色
        # 绘制瞄准判定阈值圆
        cv2.circle(im0, (int(aim_x), int(aim_y)), int(AIM_PIXEL_THRESHOLD), (255, 0, 0), 1)
        cv2.putText(im0, f"补偿后: ({int(aim_x)}, {int(aim_y)})",
                   (int(aim_x) + 10, int(aim_y) + 20),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)

        # 显示靶点到落点的像素距离
        distance_color = (0, 255, 0) if pixel_distance <= AIM_PIXEL_THRESHOLD else (0, 0, 255)
        cv2.putText(im0, f"像素距离: {pixel_distance:.1f}px",
                   (int(target_x) + 10, int(target_y) - 10),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, distance_color, 2)

        # 距离信息
        distance_text = f"距离: {distance:.1f}cm" if distance else "距离: 未知"
        cv2.putText(im0, distance_text, (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)

    def publish_adjustment(self, delta_h, delta_v):
        adjust_msg = Twist()
        adjust_msg.linear.x = delta_h
        adjust_msg.linear.y = delta_v
        self.adjust_pub.publish(adjust_msg)

    def adjust_pid_by_distance(self, distance):
        # 针对远距离优化PID参数（更平滑的过渡）
        if distance and distance > 500:
            # 远距离：适度增加灵敏度，但不要过于激进
            self.pid_h['Kp'] = 0.55  # 从0.5增加到0.55
            self.pid_v['Kp'] = 0.65  # 从0.6增加到0.65
            self.PID_MAX_INCREMENT = 3.2  # 适度增加最大调整量
        elif distance and distance > 300:
            # 中距离：轻微调整
            self.pid_h['Kp'] = 0.52
            self.pid_v['Kp'] = 0.62
            self.PID_MAX_INCREMENT = 2.8
        elif distance and distance > 200:
            # 中近距离：保持默认
            self.pid_h['Kp'] = 0.5
            self.pid_v['Kp'] = 0.6
            self.PID_MAX_INCREMENT = 2.5
        else:
            # 近距离：稍微降低灵敏度，提高稳定性
            self.pid_h['Kp'] = 0.48
            self.pid_v['Kp'] = 0.58
            self.PID_MAX_INCREMENT = 2.0

    def pid_controller(self, err, pid_params):
        # 限制积分项累积，避免过大调整
        pid_params['err_sum'] = np.clip(pid_params['err_sum'] + err, -3, 3)
        derr = err - pid_params['last_err']
        pid_params['last_err'] = err

        output = (pid_params['Kp'] * err +
                 pid_params['Ki'] * pid_params['err_sum'] +
                 pid_params['Kd'] * derr)
        return np.clip(output, -self.PID_MAX_INCREMENT, self.PID_MAX_INCREMENT)

    def gun_callback(self, req):
        response = TriggerResponse()
        try:
            # 每次服务调用前重置所有状态
            self.reset_for_new_target()

            # 标记为活跃状态（开始处理）
            self.active = True

            # 主处理循环（仅在active状态下运行）
            for path, im, im0s, vid_cap, s in self.dataset:
                # 如果已命中目标，退出循环
                if not self.active:
                    break

                im = torch.from_numpy(im).to(self.model.device)
                im = im.half() if self.model.fp16 else im.float()
                im /= 255.0
                if len(im.shape) == 3:
                    im = im[None]

                im0, target_info = self.detect_and_process_target(im, im0s[0].copy())
                self.process_control_logic(target_info)

                # 已禁用界面显示 - 摄像头在后台静默工作
                # if self.view_img:
                #     cv2.imshow(self.window_name, im0)
                #     if cv2.waitKey(1) & 0xFF == ord('q'):
                #         print("用户退出")
                #         self.active = False
                #         break

            response.success = True
            response.message = "打靶流程完成，等待下一次指令"
            print("\n===== 本次打靶流程已完成，等待下一次服务调用 =====")

        except Exception as e:
            print(f"错误: {str(e)}")
            response.success = False
            response.message = f"错误: {str(e)}"

        return response

    def reset_for_new_target(self):
        """重置系统状态，为新目标做准备"""
        print("\n===== 重置系统，准备新的打靶流程 =====")

        # 停止任何开火动作
        self.send_gun_command(False)

        # 完全重置所有状态（包括距离）
        self.distance_history.clear()
        self.fixed_distance = None  # 修复：应该是 None，不是 False
        self.distance_fixed = False
        self.has_dataset_match = False
        self.distance_update_counter = 0  # 重置距离更新计数器

        # 重置目标跟踪相关参数
        self.target_history.clear()
        self.stable_counter = 0
        self.target_lost_counter = 0

        # 重置控制状态
        self.gun_state = "idle"
        self.is_adjusting = False
        self.reset_pid()

        # 重置目标获取状态
        self.previous_target_exists = False
        self.target_acquired = False
        self.active = False  # 初始为非活跃

        # 等待一段时间确保状态重置完成
        rospy.sleep(0.5)  # 减少等待时间

        print("===== 系统重置完成，等待新的服务调用 =====")

    def process_control_logic(self, target_info):
        current_time = time.time()

        # 如果已结束活跃状态，不执行任何逻辑
        if not self.active:
            return

        # 检测目标从有到无的状态变化（命中检测）
        current_target_exists = (target_info is not None)

        # 情况1: 之前有靶点，现在没有靶点 → 判定为命中
        if (self.previous_target_exists and not current_target_exists and
            self.target_acquired and self.gun_state == "firing"):
            # 只有在开火状态下目标消失才判定为命中
            print("===== 开火状态下目标消失，判定为命中 =====")
            self.gun_state = "hit_detected"
            self.active = False  # 标记为非活跃，结束本次流程
            self.send_gun_command(False)  # 确保停止发射
            # 新增：发布打靶成功消息到/gun_success话题
            self.success_pub.publish(String("success"))
            print("已发布打靶成功消息到 /gun_success 话题")
            return

        # 情况2: 现在有目标，但之前没有 → 新目标出现
        if current_target_exists and not self.previous_target_exists and not self.target_acquired:
            print("===== 检测到新目标出现 =====")
            self.target_acquired = True

        # 更新目标存在状态
        self.previous_target_exists = current_target_exists

        # 如果处于命中检测状态，不执行其他逻辑
        if self.gun_state == "hit_detected":
            return

        # 处理目标信息
        if target_info:
            # 重置目标丢失计数器
            self.target_lost_counter = 0

            angle_err_h = target_info['angle_err_h']
            angle_err_v = target_info['angle_err_v']
            distance = target_info['distance']
            in_center = target_info['in_center']
            pixel_distance = target_info['pixel_distance']
            aim_point = target_info['aim_point']

            # 打印像素距离信息
            print(f"靶点到落点像素距离: {pixel_distance:.1f}px, 判定: {'已瞄准' if in_center else '未瞄准'}")
            print(f"调试: 阈值={AIM_PIXEL_THRESHOLD}, 距离={distance:.1f}cm, 稳定计数={self.stable_counter}/{self.STABLE_THRESHOLD}")

            # 舵机调整逻辑
            if (not in_center and not self.is_adjusting
                and self.gun_state not in ["firing", "cooling"]
                and current_time - self.last_adjust_time > self.MIN_ADJUST_INTERVAL):

                print(f"\n===== 开始调整 =====")
                print(f"距离: {distance:.1f}cm, 水平偏差: {angle_err_h:.2f}°, 垂直偏差: {angle_err_v:.2f}°")
                print(f"目标: 移动到瞄准点 ({aim_point[0]:.1f}, {aim_point[1]:.1f})")

                self.adjust_pid_by_distance(distance)
                delta_h = self.pid_controller(angle_err_h, self.pid_h)
                delta_v = self.pid_controller(angle_err_v, self.pid_v)

                # 远距离时降低调整阈值，提高灵敏度
                min_delta_h = 0.1 if (distance and distance > 500) else 0.15
                min_delta_v = 0.08 if (distance and distance > 500) else 0.1

                if abs(delta_h) < min_delta_h:
                    delta_h = 0
                if abs(delta_v) < min_delta_v:
                    delta_v = 0

                if delta_h != 0 or delta_v != 0:
                    self.publish_adjustment(delta_h, delta_v)

                    target_h = self.servo_horizontal - delta_h
                    target_v = self.servo_vertical - delta_v

                    if self.check_serial_connection():
                        self.last_adjust_time = current_time
                        self.is_adjusting = True
                        self.gun_state = "adjusting"

                        if abs(delta_h) > min_delta_h:
                            success = self.move_servo(target_h, SERVO_HORIZONTAL)
                            if success:
                                self.servo_horizontal = target_h

                        if abs(delta_v) > min_delta_v:
                            success = self.move_servo(target_v, SERVO_VERTICAL)
                            if success:
                                self.servo_vertical = target_v

            # 调整状态管理
            if self.is_adjusting:
                if current_time - self.last_adjust_time > self.ADJUST_SETTLE_TIME:
                    self.is_adjusting = False
                    # 只有在调整状态时才转换状态
                    if self.gun_state == "adjusting":
                        if self.distance_fixed:
                            self.gun_state = "ready"
                            print(f"===== 调整完成，距离已固定，准备开火 =====")
                        else:
                            self.gun_state = "stabilizing"
                            print(f"===== 调整完成，等待距离固定 =====")

            # 自动开火逻辑
            if self.distance_fixed:  # 距离固定后才允许开火
                # 修复：如果当前状态是 "idle" 且目标已瞄准，直接转为 "ready"
                if self.gun_state == "idle" and in_center:
                    self.gun_state = "ready"
                    print(f"===== 目标已瞄准，距离已固定，状态转为准备开火 =====")

                # 如果当前状态是 "stabilizing"，距离固定后转为 "ready"
                if self.gun_state == "stabilizing":
                    self.gun_state = "ready"
                    print(f"===== 距离已固定，状态转为准备开火 =====")

                # 处理不同状态下的逻辑
                if self.gun_state == "ready":
                    if in_center:
                        # 目标在落点区域内，累积稳定计数
                        self.stable_counter += 1
                        print(f"目标稳定在落点区域: {self.stable_counter}/{self.STABLE_THRESHOLD}")

                        # 稳定计数达到阈值且不在冷却中，开始开火
                        if (self.stable_counter >= self.STABLE_THRESHOLD and
                            current_time - self.last_fire_end_time > self.COOLDOWN_DURATION):

                            self.gun_state = "firing"
                            self.fire_start_time = current_time
                            self.send_gun_command(True)
                            print(f"===== 开始开火 =====")
                    else:
                        # 目标偏离落点区域，重置稳定计数
                        print("目标偏离落点区域，重置稳定计数")
                        self.stable_counter = 0

                elif self.gun_state == "stabilizing":
                    # 在稳定状态下，如果目标进入瞄准区域，可以提前转为ready
                    if in_center:
                        print("目标进入瞄准区域，提前转为准备状态")
                        self.gun_state = "ready"
                        self.stable_counter = 1  # 给予一些初始稳定计数

                # 开火状态管理
                if self.gun_state == "firing":
                    # 检查是否达到最大开火时间
                    if current_time - self.fire_start_time >= self.FIRE_DURATION:
                        self.send_gun_command(False)
                        self.gun_state = "cooling"
                        self.last_fire_end_time = current_time
                        # 开火结束后重置稳定计数，但保留目标获取状态
                        self.stable_counter = 0
                        print(f"===== 开火时间结束，停止开火 =====")

                # 冷却状态管理
                if self.gun_state == "cooling":
                    if current_time - self.last_fire_end_time >= self.COOLDOWN_DURATION:
                        self.gun_state = "ready"
                        print(f"===== 冷却完成，准备再次开火 =====")

        else:
            # 目标丢失时的处理
            if not self.target_acquired:
                return

            # 重置调整状态和目标历史
            self.is_adjusting = False
            self.target_history.clear()

            # 如果正在开火，增加目标丢失计数器
            if self.gun_state == "firing":
                self.target_lost_counter += 1
                print(f"开火中目标丢失: {self.target_lost_counter}/{self.TARGET_LOST_THRESHOLD}")

                # 如果连续丢失目标达到阈值，停止开火
                if self.target_lost_counter >= self.TARGET_LOST_THRESHOLD:
                    self.send_gun_command(False)
                    self.gun_state = "cooling"
                    self.last_fire_end_time = current_time
                    # 开火失败时重置稳定计数
                    self.stable_counter = 0
                    print(f"===== 目标丢失超过阈值，判定未命中，停止开火 =====")

            elif self.gun_state in ["ready", "stabilizing"]:
                # 在准备状态或稳定状态时目标丢失，重置稳定计数
                print("目标丢失，重置稳定计数")
                self.stable_counter = 0
                self.gun_state = "idle"
                self.reset_pid()

            elif self.gun_state not in ["idle", "cooling"]:
                # 处理其他状态（如 adjusting）
                if self.gun_state == "firing":
                    self.send_gun_command(False)
                    self.last_fire_end_time = current_time
                self.gun_state = "idle"
                self.reset_pid()
                print("目标丢失，停止所有动作")

    def reset_pid(self):
        self.pid_h['err_sum'] = 0
        self.pid_h['last_err'] = 0
        self.pid_v['err_sum'] = 0
        self.pid_v['last_err'] = 0

    def run(self):
        try:
            rospy.spin()
        except KeyboardInterrupt:
            print("\nROS节点被中断，正在清理资源...")
            self.cleanup_resources()
        except Exception as e:
            print(f"ROS节点异常: {e}")
            self.cleanup_resources()

    def cleanup_resources(self):
        """清理所有资源"""
        try:
            print("正在清理资源...")

            # 关闭串口
            if hasattr(self, 'ser') and self.ser and self.ser.is_open:
                self.ser.close()
                print("串口已关闭")

            print("资源清理完成")

        except Exception as e:
            print(f"清理资源时出错: {e}")


# 主程序入口
def parse_opt():
    parser = argparse.ArgumentParser()
    parser.add_argument("--weights", nargs="+", type=str, default="best.pt", help="模型路径")
    parser.add_argument("--source", type=str, default="0", help="输入源")
    parser.add_argument("--data", type=str, default="data/coco128.yaml", help="数据集配置")
    parser.add_argument("--imgsz", "--img", "--img-size", nargs="+", type=int, default=[640], help="推理尺寸")
    parser.add_argument("--conf-thres", type=float, default=0.4, help="置信度阈值")
    parser.add_argument("--iou-thres", type=float, default=0.5, help="NMS阈值")
    parser.add_argument("--max-det", type=int, default=5, help="最大检测数")
    parser.add_argument("--device", default="", help="计算设备")
    parser.add_argument("--view-img", action="store_false", help="不显示图像")
    parser.add_argument("--save-txt", action="store_true", help="保存txt")
    parser.add_argument("--nosave", action="store_true", help="不保存结果")
    parser.add_argument("--classes", nargs="+", type=int, help="过滤类别")
    parser.add_argument("--agnostic-nms", action="store_true", help="类别无关NMS")
    parser.add_argument("--augment", action="store_true", help="增强推理")
    parser.add_argument("--project", default=Path("runs/detect"), help="保存目录")
    parser.add_argument("--name", default="exp", help="实验名")
    parser.add_argument("--exist-ok", action="store_true", help="允许覆盖")
    parser.add_argument("--line-thickness", default=3, type=int, help="框线宽")
    parser.add_argument("--half", action="store_true", help="半精度推理")
    parser.add_argument("--dnn", action="store_true", help="使用DNN推理")
    parser.add_argument("--port", type=str, default="/dev/gun", help="串口路径")
    parser.add_argument("--baudrate", type=int, default=115200, help="波特率")
    opt = parser.parse_args()
    opt.imgsz *= 2 if len(opt.imgsz) == 1 else 1
    return opt

def main(opt):
    print(f"使用串口: {opt.port}，波特率: {opt.baudrate}")
    print(f"舵机角度限制：水平{ HORIZONTAL_MIN }-{ HORIZONTAL_MAX }度，垂直{ VERTICAL_MIN }-{ VERTICAL_MAX }度")
    print(f"瞄准判定阈值: 靶点距离目标落点≤{AIM_PIXEL_THRESHOLD}像素")

    # 先初始化ROS节点
    rospy.init_node('gun_controller', anonymous=True, log_level=rospy.WARN)

    controller = GunControllerNode(opt)
    controller.init_camera()

    try:
        controller.run()
    except KeyboardInterrupt:
        print("\n程序被用户中断，正在清理资源...")
    except Exception as e:
        print(f"程序异常: {e}")
    finally:
        # 程序结束时统一清理资源
        print("正在清理资源...")
        if hasattr(controller, 'cleanup_resources'):
            controller.cleanup_resources()
        print("资源清理完成")

if __name__ == "__main__":
    opt = parse_opt()
    main(opt)
