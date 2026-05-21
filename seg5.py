import rclpy
import math
import numpy as np
import time
import threading

from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

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
# Robot Controller
# ==========================================
class Robot_Ctrl(object):
    def __init__(self):
        self.rec_thread = Thread(target=self.rec_responce)
        self.send_thread = Thread(target=self.send_publish)

        self.lc_r = lcm.LCM("udpm://239.255.76.67:7670?ttl=255")
        self.lc_s = lcm.LCM("udpm://239.255.76.67:7671?ttl=255")

        self.cmd_msg = robot_control_cmd_lcmt()
        self.send_lock = Lock()
        self.delay_cnt = 0
        self.runing = 1

    def run(self):
        self.lc_r.subscribe("robot_control_response", self.msg_handler)
        self.send_thread.start()
        self.rec_thread.start()

    def msg_handler(self, channel, data):
        pass

    def rec_responce(self):
        while self.runing:
            self.lc_r.handle()
            time.sleep(0.002)

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
# Coordinate Transform
# ==========================================
def world_to_robot(dx, dy, yaw):
    fx = math.cos(yaw) * dx + math.sin(yaw) * dy
    fy = -math.sin(yaw) * dx + math.cos(yaw) * dy
    return fx, fy


# ==========================================
# MAIN — 9 WAYPOINTS + FULL ROLL CONTROL
# ==========================================
def main():
    rclpy.init()

    pose = SimPoseSubscriber()
    threading.Thread(target=lambda: rclpy.spin(pose), daemon=True).start()
    time.sleep(1)

    ctrl = Robot_Ctrl()
    ctrl.run()

    msg = robot_control_cmd_lcmt()
    pid_yaw = PID(2.5, 0.0, 0.1)

    # --------------------------
    # YOUR FULL 9 WAYPOINTS
    # --------------------------
    WAYPOINTS = [
        (3.12, 12.2, 1.58),    # WP1 flat
        (3.12, 12.3, 3.14),    # WP2 flat
        (-0.29, 12.5, 3.14),   # WP3 slope
        (-0.29, 12.5, 1.56),    # WP4 slope end
        (-0.29,15.2 ,1.56),
        
        (-0.35,15.3 ,0),

        
        (3.13, 15.2, 0),   # WP9 flat
        (3.12,15.2, -1.54),
        (3.12,13.4, -1.54),

    ]

    # ROLL SETTINGS
    ROLL_FLAT = 0.0
    ROLL_SLOPE =- math.radians(15)

    msg.life_count = 0

    # --------------------------
    # STAND
    # --------------------------
    print("[1] STAND")
    msg.mode = 12
    msg.gait_id = 0
    msg.vel_des = [0.0, 0.0, 0.0]
    msg.rpy_des = [0.0, 0.0, 0.0]
    msg.life_count = (msg.life_count + 1) % 128
    ctrl.Send_cmd(msg)
    time.sleep(2)

    # --------------------------
    # WALK
    # --------------------------
    print("[2] WALK GAIT 26")
    msg.mode = 11
    msg.gait_id = 26
    msg.step_height = [0.09, 0.09]
    msg.vel_des = [0.0, 0.0, 0.0]
    msg.rpy_des = [0.0, 0.0, 0.0]
    msg.life_count = (msg.life_count + 1) % 128
    ctrl.Send_cmd(msg)
    time.sleep(1.5)

    # --------------------------
    # TRACK ALL WAYPOINTS
    # --------------------------
    for idx, (gx, gy, gyaw) in enumerate(WAYPOINTS):
        print(f"\n===== WP{idx+1}: ({gx:.2f}, {gy:.2f}), yaw={gyaw:.2f} =====")

        # ROLL RULE 100% AS YOU WANT
        if idx in [0, 1, 7, 8]:
            current_roll = ROLL_FLAT
        else:
            current_roll = ROLL_SLOPE

        while True:
            x, y, yaw = pose.get()
            dx = gx - x
            dy = gy - y

            fx, fy = world_to_robot(dx, dy, yaw)
            dist = math.hypot(dx, dy)
            norm = math.hypot(fx, fy) + 1e-6

            vx = 0.10 * fx / norm if dist > 0.1 else 0.0
            vy = 0.0
            wz = pid_yaw.update(gyaw - yaw)

            # Send control
            msg.mode = 11
            msg.gait_id = 26
            msg.vel_des = [vx, vy, wz]
            msg.rpy_des = [current_roll, -20.0, 0.0]

            # NEVER FORGET MODULO 128 AGAIN
            msg.life_count = (msg.life_count + 1) % 128
            ctrl.Send_cmd(msg)

            # Reached target
            if dist < 0.1 and abs(pid_yaw.wrap(gyaw - yaw)) < 0.15:
                break

            time.sleep(0.05)

    # --------------------------
    # FINISH
    # --------------------------
    print("\n✅ ALL WAYPOINTS FINISHED")
    msg.mode = 7
    msg.gait_id = 0
    msg.vel_des = [0.0, 0.0, 0.0]
    msg.rpy_des = [0.0, 0.0, 0.0]
    msg.life_count = (msg.life_count + 1) % 128
    ctrl.Send_cmd(msg)
    time.sleep(2)

    ctrl.quit()
    rclpy.shutdown()


if __name__ == "__main__":
    main()