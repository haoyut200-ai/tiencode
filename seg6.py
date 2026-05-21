import rclpy
import cv
import math
import numpy as np
import time
import threading
from threading import Thread, Lock

from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

import lcm
from robot_control_cmd_lcmt import robot_control_cmd_lcmt
from robot_control_response_lcmt import robot_control_response_lcmt


# ==========================================
# PID Controller
# ==========================================
class PID:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.i = 0.0
        self.prev = 0.0
        self.t_prev = time.time()

    def wrap(self, a):
        return (a + math.pi) % (2 * math.pi) - math.pi

    def update(self, err):
        t = time.time()
        dt = max(t - self.t_prev, 1e-6)
        self.t_prev = t

        err = self.wrap(err)
        self.i += err * dt
        d = (err - self.prev) / dt
        self.prev = err

        return self.kp * err + self.ki * self.i + self.kd * d


# ==========================================
# POSE Subscriber
# ==========================================
class SimPoseSubscriber(Node):
    def __init__(self):
        super().__init__('pose')
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.sub = self.create_subscription(PoseStamped, '/simulator_pose', self.cb, qos)
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

    def quat2yaw(self, x, y, z, w):
        return np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))

    def cb(self, msg):
        q = msg.pose.orientation
        self.yaw = self.quat2yaw(q.x, q.y, q.z, q.w)
        self.x = msg.pose.position.x
        self.y = msg.pose.position.y

    def get(self):
        return self.x, self.y, self.yaw


# ==========================================
# Robot Controller
# ==========================================
class RobotCtrl:
    def __init__(self):
        self.lc_r = lcm.LCM("udpm://239.255.76.67:7670?ttl=255")
        self.lc_s = lcm.LCM("udpm://239.255.76.67:7671?ttl=255")

        self.cmd_msg = robot_control_cmd_lcmt()
        self.rec_msg = robot_control_response_lcmt()

        self.send_lock = Lock()
        self.delay_cnt = 0

        self.mode_ok = 0
        self.gait_ok = 0
        self.running = True

        self.rec_thread = Thread(target=self.rec_response)
        self.send_thread = Thread(target=self.send_publish)

    def run(self):
        self.lc_r.subscribe("robot_control_response", self.msg_handler)
        self.send_thread.start()
        self.rec_thread.start()

    def msg_handler(self, channel, data):
        self.rec_msg = robot_control_response_lcmt().decode(data)
        if self.rec_msg.order_process_bar >= 95:
            self.mode_ok = self.rec_msg.mode
            self.gait_ok = self.rec_msg.gait_id
        else:
            self.mode_ok = 0
            self.gait_ok = 0

    def rec_response(self):
        while self.running:
            self.lc_r.handle()
            time.sleep(0.002)

    def send_publish(self):
        while self.running:
            with self.send_lock:
                if self.delay_cnt > 20:
                    self.lc_s.publish("robot_control_cmd", self.cmd_msg.encode())
                    self.delay_cnt = 0
                self.delay_cnt += 1
            time.sleep(0.005)

    def send_cmd(self, msg):
        with self.send_lock:
            self.delay_cnt = 50
            self.cmd_msg = msg

    def wait_finish(self, target_mode, target_gait, timeout_sec=8.0):
        print(f"[Wait] Waiting for mode {target_mode}, gait {target_gait}...")
        timeout = time.time() + timeout_sec
        while time.time() < timeout:
            if self.mode_ok == target_mode and self.gait_ok == target_gait:
                print(f"[Success] Reached mode {target_mode}, gait {target_gait}")
                return True
            time.sleep(0.1)
        print("[Warning] Wait timeout")
        return False

    def quit(self):
        self.running = False
        self.rec_thread.join()
        self.send_thread.join()


# ==========================================
# 辅助函数
# ==========================================
def world_to_robot(dx, dy, yaw):
    fx = math.cos(yaw) * dx + math.sin(yaw) * dy
    fy = -math.sin(yaw) * dx + math.cos(yaw) * dy
    return fx, fy

def move_to_target(pose_sub, ctrl, pid_yaw, msg, target_x, target_y, target_yaw, 
                   speed=0.3, dist_tol=0.12, yaw_tol=0.05, custom_vx=None):
    """
    通用导航到目标点的循环控制函数，大大减少代码冗余
    """
    while True:
        x, y, yaw = pose_sub.get()
        dx, dy = target_x - x, target_y - y
        fx, fy = world_to_robot(dx, dy, yaw)
        norm = math.hypot(fx, fy) + 1e-6

        # 允许通过 custom_vx 覆盖自动计算的 x 轴速度（如第3步持续顶球场景）
        vx = custom_vx if custom_vx is not None else (speed * fx / norm)
        vy = speed * fy / norm
        wz = pid_yaw.update(target_yaw - yaw)

        msg.vel_des = [vx, vy, wz]
        msg.life_count = (msg.life_count + 1) % 127
        ctrl.send_cmd(msg)

        # 满足距离和角度条件后退出
        if math.hypot(dx, dy) < dist_tol and abs(wz) < yaw_tol:
            return
        time.sleep(0.05)


# ==========================================
# MAIN 任务流
# ==========================================
def main():
    rclpy.init()

    pose = SimPoseSubscriber()
    threading.Thread(target=lambda: rclpy.spin(pose), daemon=True).start()
    time.sleep(1)

    ctrl = RobotCtrl()
    ctrl.run()
    msg = robot_control_cmd_lcmt()
    pid_yaw = PID(2.2, 0.0, 0.12)

    # 1. 复位站立
    print("\n[1] 复位站立: mode=12, gait=0")
    msg.mode = 12
    msg.gait_id = 0
    msg.life_count += 1
    ctrl.send_cmd(msg)
    ctrl.wait_finish(12, 0)
    time.sleep(0.5)

    # 2. 下楼模式初始化
    print("\n[2] 下楼模式启动: mode=11, gait=8")
    msg.mode = 11
    msg.gait_id = 8
    msg.vel_des = [0.0, 0, 0]
    msg.rpy_des = [0, -0.16, 0]
    msg.step_height = [0.012, 0.012]
    msg.life_count += 1
    msg.duration = 0
    ctrl.send_cmd(msg)
    ctrl.wait_finish(11, 8)

    # --------------------------
    # 第1步：走到白球正后方
    # --------------------------
    print("\n→ 第1步：走到白球正后方")
    move_to_target(pose, ctrl, pid_yaw, msg, target_x=0.14, target_y=14.1, target_yaw=0.1, speed=0.3)
    print("✅ 已到达白球正后方")

    # --------------------------
    # 新增步：移动到截图坐标 + 逆时针旋转90°
    # --------------------------
    print("\n→ 新增步骤：移动到指定坐标，并逆时针旋转90度")
    move_to_target(pose, ctrl, pid_yaw, msg, target_x=0.145571, target_y=14.691593, target_yaw=math.pi/2, 
                   speed=0.25, dist_tol=0.08, yaw_tol=0.04)
    print("✅ 逆时针旋转90°完成")

    # --------------------------
    # 第2步：向右转向+低速走到截图坐标，屁股下拉卡住球前进
    # --------------------------
    print("\n→ 第2步：低速前进，屁股下拉卡住球")
    msg.rpy_des = [0, -0.22, 0] # 持续下压屁股
    move_to_target(pose, ctrl, pid_yaw, msg, target_x=0.582434, target_y=14.745421, target_yaw=0.096349, 
                   speed=0.15, dist_tol=0.12, yaw_tol=0.04)
    print("✅ 已到达目标位置，卡住球")

    # --------------------------
    # 第3步：向右顶球走到出口处
    # --------------------------
    print("\n→ 第3步：向右顶球走到出口")
    # 此步骤中原代码的 vx 是固定的 0.3，我们通过 custom_vx 参数传入，并放宽角度限制
    move_to_target(pose, ctrl, pid_yaw, msg, target_x=2.59, target_y=14.9, target_yaw=0.1, 
                   speed=0.10, dist_tol=0.15, yaw_tol=1.0, custom_vx=0.30)
    print("✅ 已将球顶到出口处")

    # --------------------------
    # 第4步：移到白球下方，朝向朝上
    # --------------------------
    print("\n→ 第4步：移到白球下方，朝向朝上")
    move_to_target(pose, ctrl, pid_yaw, msg, target_x=2.17, target_y=12.9, target_yaw=0.25, speed=0.05)
    print("✅ 已到达白球下方，朝向朝上")

    # --------------------------
    # 第5步：向前将白球踢出U型口 (已修复 Bug)
    # --------------------------
    print("\n→ 第5步：向前踢出白球")
    # 修复了原代码中 if x < 0.2 的 Bug，统一使用距离容差判断
    move_to_target(pose, ctrl, pid_yaw, msg, target_x=3.2, target_y=12.9, target_yaw=0.1, speed=0.1)
    print("✅ 白球已成功踢出U型口")

    # --------------------------
    # 第6步：阻尼卧倒
    # --------------------------
    print("\n→ 第6步：阻尼卧倒结束")
    msg.mode = 7
    msg.gait_id = 0
    msg.vel_des = [0.0, 0.0, 0.0]
    msg.life_count += 1
    ctrl.send_cmd(msg)
    time.sleep(2.0)

    # 退出程序
    ctrl.quit()
    rclpy.shutdown()
    print("\n🎉 全部任务完成！")

if __name__ == "__main__":
    main()