#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Historical 2025 mission route, cleaned for repository display.

The waypoint sequence is preserved from the vehicle backup. Device-specific
paths are now derived from the repository or environment variables. This file
has not been revalidated on the original vehicle after archival cleanup.
"""

from datetime import datetime
from pathlib import Path
from geometry_msgs.msg import Pose, Point, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
import rospy
from ultralytics import YOLO
import cv2
import os
import shutil
from dynamic_reconfigure.client import Client
from nav_demo.srv import GunSweep, GunSweepRequest
from geometry_msgs.msg import Twist
import threading


PARAMS_A = {
    # DWA 参数
    'max_vel_x' : 0.3,
    'min_vel_x' : 0,
    'max_vel_y' : 0.3,
    'min_vel_y' : 0.0,
    'max_rot_vel' : 0.5,
    'min_rot_vel' : 0,
    'acc_lim_x' : 1.5,
    'acc_lim_y' : 1.5,
    'acc_lim_th' : 1.9,
    'inflation_radius': 0.01,
    # "occdist_scale": 0.3,
}


# B区参数
PARAMS_B = {
    'max_vel_x' : 0.4,
    'min_vel_x' : 0.2,
    'max_vel_y' : 0.4,
    'min_vel_y' : 0.2,
    'max_rot_vel' : 0.2,
    'min_rot_vel' : 0,
    'acc_lim_x' : 1,
    'acc_lim_y' : 1,
    'acc_lim_th' : 1.4,
    'inflation_radius': 0.02,
}

# C区参数
PARAMS_C = {
    # DWA 参数
    'max_vel_x' : 0.3,
    'min_vel_x' : 0,
    'max_vel_y' : 0.3,
    'min_vel_y' : 0.0,
    'max_rot_vel' : 0.5,
    'min_rot_vel' : 0,
    'acc_lim_x' : 1.5,
    'acc_lim_y' : 1.5,
    'acc_lim_th' : 1.9,
    'inflation_radius': 0.01,
    "occdist_scale": 0.1,
}

PARAMS_Y = {
  "acc_lim_x": 0.5,  # maximum is theoretically 2.0
  "acc_lim_y": 0.5,  # diff drive robot
  "acc_lim_th": 2,
  "max_vel_x": 0.2,
  "min_vel_x": -0.2,
  "max_vel_y": 0.15 , #diff drive robot,don't need set vel_y
  "min_vel_y": -0.15,
  "max_rot_vel": 0.5 , #choose slightly less than the base's capability
  "min_rot_vel": 0.3 , #this is the min angular velocity when there is negligible translational velocity
  "yaw_goal_tolerance": 0.1 , # 0.1 rad = 5.7 degree
  "xy_goal_tolerance": 0.05,
  "sim_time": 1.8 ,  # 1.7
  "sim_granularity": 0.025,
  "vx_samples": 50 ,   # default 3
  "vy_samples": 1,    # diff drive robot, there is only one sample
  "vth_samples": 20 , # 20
  "path_distance_bias": 90.0 ,     # 32.0
  "goal_distance_bias": 24.0   ,   # 24.0
  "occdist_scale": 0.3   ,         # 0.01
  "forward_point_distance": 0.325 ,# 0.325
  "stop_time_buffer": 0.2 ,        # 0.2
  "scaling_speed": 0.2   ,        # 0.25
  "max_scaling_factor": 0.2 ,      # 0.2
  "oscillation_reset_dist": 0.05  ,
}
# D区参数
PARAMS_D = {
    'max_vel_x' : 0.3,
    'min_vel_x' : -0.3,
    'max_vel_y' : 0.3,
    'min_vel_y' : 0,
    'max_rot_vel': 0.7,
    'min_rot_vel' : 0,
    'acc_lim_x' : 0.5,
    'acc_lim_y' : 0.5,
    'acc_lim_th' : 0.5,
    'inflation_radius': 0.03,
    "occdist_scale": 0.3,
}

# 战区参数映射
ZONE_PARAMS = {
    'A': PARAMS_A,
    'B': PARAMS_B,
    'C': PARAMS_C,
    'D': PARAMS_D,
    'Y':PARAMS_Y
}


final_stats_str = ""
# 类别映射表
label_map = {
    0: "敌军",
    1: "敌军",
    2: "友军",
    3: "人质"
}
dwa_client = None
global_client = None

PROJECT_ROOT = Path(__file__).resolve().parents[5]
MODEL_PATH = Path(os.environ.get(
    "RAICOM_SOLDIER_MODEL",
    str(PROJECT_ROOT / "models" / "soldier" / "robot_deployed_best.pt"),
))
CAMERA_INDEX = int(os.environ.get("RAICOM_CAMERA_INDEX", "0"))
model = YOLO(str(MODEL_PATH))
output_folder = str(
    PROJECT_ROOT / "artifacts" / "mission" / datetime.now().strftime("%Y%m%d_%H%M%S")
)
os.makedirs(output_folder, exist_ok=True)
photo_path = None
def camm(num):
    global photo_path
    cap = cv2.VideoCapture(CAMERA_INDEX)
    if not cap.isOpened():
        print("无法打开摄像头")
        exit()
    ret, frame = cap.read()
    if not ret:
        print("无法获取图像")
        cap.release()
        exit()
    cap.release()
    photo_path = os.path.join(output_folder, "input", str(num)+"input.jpg")
    input_path = os.path.join(output_folder, "input")
    os.makedirs(input_path, exist_ok=True)
    cv2.imwrite(photo_path, frame)
    cap.release()
# model.overrides['verbose'] = False  # 添加这一行禁用verbose参数
def cam(num):
    global photo_path
    #     show_labels=True,       # 显示标签
    # print(photo_path)
    frame = cv2.imread(photo_path)
    results = model.predict(
        source=frame,           # 使用拍摄的图像作为输入
        save=True,              # 保存检测结果图片
        project=output_folder,  # 结果保存文件夹
        name=num,               # 结果子文件夹
        exist_ok=False,         # 允许覆盖现有结果
        show_labels=True,       # 显示标签
        show_conf=True,          # 显示置信度
        # verbose=False  # 添加这一行，明确禁用verbose输出
    )
    # results = model(frame, conf = 0.2)
    new_result_path = None
    prediction_dir = os.path.join(output_folder, str(num))
    prediction_image = os.path.join(prediction_dir, "image0.jpg")
    if os.path.exists(prediction_image):
        print("ex")
        # 新的结果图片名称，包含编号和"检测结果"标识
        new_result_name = f"out_put_{num}.jpg"
        new_result_path = os.path.join(output_folder, new_result_name)
        if os.path.exists(new_result_path):
            os.remove(new_result_path)
        shutil.move(prediction_image, new_result_path)
    if new_result_path:
        print(f"检测结果图片已保存至: {new_result_path}")
    else:
        print("未找到 Ultralytics 保存的标注图；请检查推理输出")
    print(f"\n拍摄的照片已保存至: {photo_path}")
    print("检测结果:")
    # 统计各类别数量
    counts = {
        "敌军": 0,
        "友军": 0,
        "人质": 0
    }

    # 遍历检测结果并计数
    for box in results[0].boxes:
        cls_id = int(box.cls)
        cls_name = label_map.get(cls_id, '未知')
        if cls_name in counts:
            counts[cls_name] += 1
    area_stats = f"{num}区: 敌军 {counts['敌军']}人, 人质 {counts['人质']}人, 友军 {counts['友军']}人\n"

    # 添加到最终统计字符串
    global final_stats_str
    final_stats_str += area_stats
    # 打印统计结果
    print(f"\n拍摄的照片已保存至: {photo_path}")
    print(area_stats)
    print(f"\n检测结果保存在: {os.path.join(output_folder, 'camera_detection')}")

def cam_thread(num):
    threading.Thread(target=cam, args=(num,)).start()




class GunClient:
    def __init__(self):
        # rospy.init_node('gun_client_node')

        # 等待服务可用
        rospy.wait_for_service('/gun_sweep')
        self.gun_service = rospy.ServiceProxy('/gun_sweep', GunSweep)  # 使用自定义服务类型
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        rospy.loginfo("Gun Client Node Started")

    def call_gun_service(self, min_angle=85, max_angle=95,angle=89,num=2):
        try:
            req = GunSweepRequest(min_angle=min_angle, max_angle=max_angle,angle=angle,num=num)
            # 调用服务
            rospy.loginfo("Calling /gun service...")
            response = self.gun_service(req)

            # 处理响应
            if response.success:
                rospy.loginfo("Service call succeeded: " + response.message)
            else:
                rospy.logwarn("Service call failed: " + response.message)

        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)
    def emergency_stop(self):
        """紧急停止机器人"""
        stop_cmd = Twist()  # 所有速度设为0
        self.cmd_vel_pub.publish(stop_cmd)
        rospy.loginfo("EMERGENCY STOP ACTIVATED!")




def reset_params(params):
    global dwa_client, global_client
    if dwa_client is None:
        try:
            dwa_client = Client("/move_base/DWAPlannerROS", timeout=30)
            global_client = Client("/move_base/global_costmap", timeout=30)
            rospy.loginfo("参数客户端初始化成功")
        except Exception as e:
            rospy.logerr("无法初始化参数客户端: %s", str(e))
            return

    # 获取当前配置
    try:
        dwa_config = dwa_client.get_configuration(timeout=5)
        global_config = global_client.get_configuration(timeout=5)

        # 过滤不支持的参数
        dwa_params = {k: v for k, v in params.items() if k in dwa_config}
        global_params = {k: v for k, v in params.items() if k in global_config}

        # 应用参数
        if dwa_params:
            dwa_client.update_configuration(dwa_params)
        if global_params:
            global_client.update_configuration(global_params)

    except Exception as e:
        rospy.logwarn("参数更新失败: %s", str(e))


def create_goal(x, y, z, w):
    goal = MoveBaseGoal()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.position.z = 0.0

    goal.target_pose.pose.orientation.x = 0
    goal.target_pose.pose.orientation.y = 0
    goal.target_pose.pose.orientation.z = z
    goal.target_pose.pose.orientation.w = w

    return goal

def send_goals_python(goals):
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    goal_number = len(goals)
    for i, goal in enumerate(goals):
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        client.send_goal(goal)
        str_log = "Send NO. {} Goal !!!".format(i+1)
        rospy.loginfo(str_log)

        wait = client.wait_for_result(rospy.Duration.from_sec(30.0))
        if not wait:
            str_log = "The NO. {} Goal Planning Failed for some reasons".format(i+1)
            rospy.loginfo(str_log)

        else:
            str_log = "The NO. {} Goal achieved success !!!".format(i+1)
            rospy.loginfo(str_log)
    return "Mission Finished."
import time
if __name__ == "__main__":
    try:
        rospy.init_node('send_goals_python', anonymous=True)
        # client = GunClient()
        # print("ready")
        a = input("ready")
        # client.call_gun_service()
        # camm('A')
        # cam('A')
        # cam_thread('A')
        # time.sleep(100000)

        # reset_params(ZONE_PARAMS['A'])
        # goals = [
        #     create_goal(1.17972719669, 0.007504901886, -0.00840453967084, 0.999964681233),  # point0
        #     create_goal(1.9999320507, -0.012989177704, 0.00167148517378, 0.999998603068),   # point1
        #     create_goal(2.89860524178, -0.0357068371773, -0.00401947961983, 0.999991921859), # point2

        #     create_goal(2.95443084717, 0.3521547102928, 0.69845340962, 0.715655527884),      # point3

        #     create_goal(2.756956453323, 0.64702937603, 0.999991156225, 0.00420564762598),    # point5

        #     create_goal(2.17484840393, 0.63473412991, 0.999998518908, -0.00172109916144),    # point6
        #     create_goal(2.19021434784, 0.61692500114, 0.708668924808, 0.705541178821),      # b1
        # ]
        # result = send_goals_python(goals)
        # camm('A')
        # cam_thread('A')
        # goals = [
        #     create_goal(2.19231480598, 0.651037430763, 0.999999521225, -0.000978545190365),  # point8

        #     create_goal(1.56682147503, 0.613297162056, 0.999994948784, -0.00317842822425),    # point9
        #     create_goal(1.60613377094, 0.6224405646, 0.704364870867, 0.709838100336),         # d1 y=0.61
        # ]
        # result = send_goals_python(goals)
        # # client.call_gun_service(85,95,85,2)
        # goals = [
        #     create_goal(1.60207029819, 0.62152050972, 0.999991910018, -0.00402242449877),     # point11

        #     create_goal(1.080377249718, 0.652417497635, 0.99999944328, -0.0010551967239),    # point12
        #     create_goal(1.07367002487, 0.652698018551, 0.710103835618, 0.704096969628),      # b2
        # ]
        # result = send_goals_python(goals)
        # camm('B')
        # cam_thread('B')

        # goals = [
        #     create_goal(1.06385882378, 0.664261806011, 0.9999844328, -0.0010551967239),      # point14

        #     create_goal(0.260266225338, 0.655613965988, 0.999991156225, 0.00420564762598),  # point15
        #     create_goal(0.10010327005, 0.75644064903, 0.714185400079, 0.699956580306),     # point15.5 x=0.08
        # ]
        # result = send_goals_python(goals)

        # goals = [
        #     create_goal(0.198435059786, 1.4473964262, 0.697362173896, 0.716718911721),      # s0

        #     create_goal(0.432807685137, 2.10252929688, -0.0119587713099, 0.999928491338),   # s4

        #     create_goal(1.08793612957, 2.10493755341, 0.0102823799862, 0.999947134934),     # d2
        # ]
        # result = send_goals_python(goals)
        # reset_params(ZONE_PARAMS['C'])
        # # client.call_gun_service(75,83,89,2)
        # goals = [
        #     create_goal(1.07726720333, 2.10357162476, 0.6, 0.8),      # s6
        #     create_goal(1.14000313759, 2.83958721161, 0.583717077821, 0.811957125136),      # s7
        #     create_goal(1.28182701111, 2.90590716362, 0.397508200876, 0.917598621531),     # s9
        #     create_goal(1.86521903038, 3.14256544113, 0.00400823771559, 0.999991966983),   # s10wandaozhong

        #     create_goal(2.31986047745, 2.535816288, -0.709965329299, 0.704236630114),      # s11.5
        #     create_goal(2.35236852646, 2.12219204903, -0.710574006365, 0.703622470845),    # s12

        #     create_goal(2.300681343079, 2.12047045517, -0.0524765876088, 0.998622154648),   # s13
        # ]
        # result = send_goals_python(goals)

        reset_params(ZONE_PARAMS['A'])

        goals=[
            # create_goal(2.8408495147705, 2.16446247101, -0.044107469074, 0.999026792019),  # s14 x=2.91
            # create_goal(2.945147705, 2.30446247101,0.4,0.9),      # s1
            # create_goal(3.08495147705, 2.40446247101, 0.702933588284, 0.71125548888),      # s15 y=2.18
            create_goal(2.98982788086, 3.42612171173, 0.708527778979, 0.705682922009),    # m-0 x=3.12 guodudian
            create_goal(3.03858539581, 4.33692474365, 0.716453823909, 0.697634516209),    # m-1
            create_goal(2.9058539581, 4.44692474365, 1,0),    # m-2
            create_goal(2.4348890305, 4.40119581223, 0.999978034146, -0.00662806346614), # m-3
            create_goal(2.3532664871, 4.40038322449, -0.705686035875, 0.708524677602),   # db3
        ]

        result = send_goals_python(goals)
        camm('C')
        cam_thread('C')
        # client.call_gun_service(95,103,85,2)

        goals = [
             create_goal(2.44810938835, 4.33778476715, 0.999956704767, -0.00930529913692),  # db回
              create_goal(1.050329566, 4.30143785477, 0.999968970428, -0.0078777014977),     # b4前
             create_goal(1.10941529274, 4.33582735062, -0.711722693847, 0.702460537726),    # b4

                ]
        result = send_goals_python(goals)
        camm('D')
        cam_thread('D')

        goals = [

             create_goal(1.13274657726, 4.26592683792, 0.999976663882, 0.00683166832144),   # b4hui
             create_goal(0.315134584904, 4.29584121704, 0.999888345364, -0.0149431190178),  # d4qian
             create_goal(0.413043439388, 4.35254240036, -0.693057639321, 0.720882173853),   # d4
                ]
        result = send_goals_python(goals)
        # client.call_gun_service(80,90,91,2)
        reset_params(ZONE_PARAMS['Y'])

        goals = [
            create_goal(0.433593899012, 4.34344768524, 0.999664241066, -0.025911486513),   # d4hui
             create_goal(0.0452519059181, 4.26882553101, -0.812935235458, 0.582354104434),   # 进入障碍区
        ]
        result = send_goals_python(goals)
        time.sleep(2)
        goals = [
            create_goal(-0.320021927357, 1.80862569809, -0.701529867432, 0.712640052973) ,   # 终点前
            create_goal(-0.280199795961, 0.053403377533, -0.691887011616, 0.722005791637)  #终点

        ]
        result = send_goals_python(goals)
        print(final_stats_str)
    except KeyboardInterrupt:
        rospy.loginfo("程序被用户中断")
        # client.emergency_stop()
        print(final_stats_str)
