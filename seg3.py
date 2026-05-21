import rclpy
import math
import numpy as np
import time
import threading

from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

import lcm
from threading import Thread, Lock

from robot_control_cmd_lcmt import robot_control_cmd_lcmt
from robot_control_response_lcmt import robot_control_response_lcmt

# ==========================================
# PID
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
# POSE
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
        self.sub = self.create_subscription(
            PoseStamped, '/simulator_pose', self.cb, qos
        )
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

    def quat2yaw(self, x, y, z, w):
        return np.arctan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))

    def cb(self, msg):
        q = msg.pose.orientation
        self.yaw = self.quat2yaw(q.x, q.y, q.z, q.w)
        self.x = msg.pose.position.x
        self.y = msg.pose.position.y

    def get(self):
        return self.x, self.y, self.yaw

# ==========================================
# LCM CONTROLLER
# ==========================================
class Robot_Ctrl(object):
    def __init__(self):
        self.rec_thread = Thread(target=self.rec_responce)
        self.send_thread = Thread(target=self.send_publish)
        self.lc_r = lcm.LCM("udpm://239.255.76.67:7670?ttl=255")
        self.lc_s = lcm.LCM("udpm://239.255.76.67:7671?ttl=255")
        self.cmd_msg = robot_control_cmd_lcmt()
        self.rec_msg = robot_control_response_lcmt()
        self.send_lock = Lock()
        self.delay_cnt = 0
        self.mode_ok = 0
        self.runing = 1

    def run(self):
        self.lc_r.subscribe("robot_control_response", self.msg_handler)
        self.send_thread.start()
        self.rec_thread.start()

    def msg_handler(self, channel, data):
        self.rec_msg = robot_control_response_lcmt().decode(data)
        if self.rec_msg.order_process_bar >= 95:
            self.mode_ok = self.rec_msg.mode

    def rec_responce(self):
        while self.runing:
            self.lc_r.handle()
            time.sleep(0.002)

    def Wait_finish(self, mode, gait_id):
        count = 0
        while self.runing and count < 2000:
            if self.mode_ok == mode:
                return True
            time.sleep(0.005)
            count += 1
        return False

    def send_publish(self):
        while self.runing:
            with self.send_lock:
                if self.delay_cnt > 20:
                    self.lc_s.publish("robot_control_cmd", self.cmd_msg.encode())
                    self.delay_cnt = 0
                self.delay_cnt += 1
            time.sleep(0.005)

    def Send_cmd(self, msg):
        with self.send_lock:
            self.delay_cnt = 50
            self.cmd_msg = msg

    def quit(self):
        self.runing = 0
        self.rec_thread.join()
        self.send_thread.join()

# ==========================================
# 坐标变换
# ==========================================
def world_to_robot(dx, dy, yaw):
    fx = math.cos(yaw) * dx + math.sin(yaw) * dy
    fy = -math.sin(yaw) * dx + math.cos(yaw) * dy
    return fx, fy

# ==========================================
# MAIN：完整20个实测弯道航点+自动转向
# ==========================================
def main():
    rclpy.init()

    pose = SimPoseSubscriber()

    def spin_all():
        exe = rclpy.executors.MultiThreadedExecutor()
        exe.add_node(pose)
        exe.spin()

    threading.Thread(target=spin_all, daemon=True).start()
    time.sleep(2)

    ctrl = Robot_Ctrl()
    ctrl.run()

    pid_yaw    = PID(2.8, 0.0, 0.12)  # 转向灵敏不甩尾
    msg = robot_control_cmd_lcmt()

    # =========================
    # STAND
    # =========================
    print("\n[1/6] STAND")
    msg.mode = 12
    msg.gait_id = 0
    msg.life_count = 1
    ctrl.Send_cmd(msg)
    ctrl.Wait_finish(12, 0)
    
    # 📍 已修改：切换时间精确为 1 秒
    time.sleep(1.0)  

    # =========================
    # WALK MODE
    # =========================
    print("\n[2/6] WALK MODE")
    msg.mode = 11
    msg.gait_id = 27
    msg.step_height = [0.09, 0.09]
    ctrl.Send_cmd(msg)
    ctrl.Wait_finish(11, 27)
    
    # 📍 已修改：将原本的 2.0 改为 1.0 秒
    time.sleep(1.0)  

    # =========================
    # SEGMENT 1
    # =========================
    print("\n[3/6] SEGMENT 1")
    while True:
        x, y, yaw = pose.get()
        if x >= 3.0:
            break
        msg.vel_des[0] = 0.15
        msg.vel_des[1] = -0.5 * y
        msg.vel_des[2] = pid_yaw.update(0 - yaw)
        msg.life_count = (msg.life_count + 1) % 127
        ctrl.Send_cmd(msg)
        time.sleep(0.05)

    # =========================
    # ALIGN YAW
    # =========================
    print("\n[4/6] ALIGN YAW")
    for _ in range(80):
        _, _, yaw = pose.get()
        msg.vel_des[0] = 0
        msg.vel_des[1] = 0
        msg.vel_des[2] = pid_yaw.update(1.56 - yaw)
        msg.life_count = (msg.life_count + 1) % 127
        ctrl.Send_cmd(msg)
        time.sleep(0.05)

    # =========================
    # SEGMENT 2
    # =========================
    print("\n[5/6] SEGMENT 2")
    WAYPOINTS = [
        (3.2, 3, 1.56),
        (2, 2, -2.7),
        (0.8, 1.3, -2.7),
        (-0.4, 3.8, 1.56),
        (-0.3, 4.6, 1.56),
    ]
    SPEED = 0.22

    for gx, gy, gyaw in WAYPOINTS:
        print(f"Go to: {gx:.1f}, {gy:.1f}, {gyaw:.2f}")
        while True:
            x, y, yaw = pose.get()
            dx, dy = gx - x, gy - y
            fx, fy = world_to_robot(dx, dy, yaw)
            norm = math.hypot(fx, fy) + 1e-6
            fx /= norm
            fy /= norm

            msg.vel_des[0] = SPEED * fx
            msg.vel_des[1] = SPEED * fy
            msg.vel_des[2] = pid_yaw.update(gyaw - yaw)
            msg.life_count = (msg.life_count + 1) % 127
            ctrl.Send_cmd(msg)

            if math.hypot(dx, dy) < 0.2 and abs(gyaw - yaw) < 0.3:
                break
            time.sleep(0.05)

    # =========================
    # SEGMENT 3: 完整20个实测弯道航点
    # =========================
    print("\n[6/6] SEGMENT 3 — 完整弯道循迹")
    LANE_WAYPOINTS = [
        # 前半段
        (-0.325997, 4.433269, 1.559895),
        (-0.308052, 4.759050, 1.559894),
        (-0.250067, 4.917240, 1.019333),
        (-0.093664, 5.097877, 1.019333),
        (-0.072115, 5.121413, 0.748857),
        (0.179803, 5.305982, 0.748857),
        (0.179807, 5.305979, 0.545710),
        (0.609582, 5.501220, 0.545710),
        (0.609588, 5.501214, 0.239101),
        (1.308711, 5.615631, 0.239101),
        # 后半段
        (1.308711, 5.615627, 0.087964),
        (2.161588, 5.725161, 0.087964),
        (2.161586, 5.725165, 0.349575),
        (2.672075, 5.990872, 0.349575),
        (2.672070, 5.990877, 0.668754),
        (2.965797, 6.302257, 0.668754),
        (2.965789, 6.302261, 1.114943),
        (3.073936, 6.618980, 1.114943),
        (3.073931, 6.618980, 1.570843),
        (3.031454, 7.062239, 1.551164),  # 终点
    ]
    SPEED_LANE = 0.16  # 弯道低速稳走
    TOLERANCE = 0.22

    for idx, (gx, gy, gyaw) in enumerate(LANE_WAYPOINTS):
        print(f"弯道航点 {idx+1}/{len(LANE_WAYPOINTS)}: X={gx:.2f}, Y={gy:.2f}, YAW={gyaw:.4f}")
        while True:
            x, y, yaw = pose.get()
            dx = gx - x
            dy = gy - y
            fx, fy = world_to_robot(dx, dy, yaw)
            dist = math.hypot(dx, dy)

            if dist < TOLERANCE:
                break

            norm = math.hypot(fx, fy) + 1e-6
            fx /= norm
            fy /= norm

            msg.vel_des[0] = SPEED_LANE * fx
            msg.vel_des[1] = SPEED_LANE * fy
            msg.vel_des[2] = pid_yaw.update(gyaw - yaw)
            msg.life_count = (msg.life_count + 1) % 127
            ctrl.Send_cmd(msg)
            time.sleep(0.05)

    # =========================
    # STOP (纯阻尼模式 mode = 7)
    # =========================
    print("\n✅ 全部航点走完，切换到纯阻尼模式")
    
    msg.vel_des[0] = 0.0
    msg.vel_des[1] = 0.0
    msg.vel_des[2] = 0.0
    
    msg.mode = 7  
    msg.life_count = (msg.life_count + 1) % 127
    ctrl.Send_cmd(msg)
    
    if ctrl.Wait_finish(7, 0):
        print("✅ 成功进入阻尼模式，机器狗已停止。")
    else:
        print("⚠️ 阻尼模式切换超时。")

    ctrl.quit()
    rclpy.shutdown()

if __name__ == "__main__":
    main()