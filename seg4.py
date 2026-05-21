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
# Robot Controller (UNCHANGED CORE)
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
        self.gait_ok = 0
        self.runing = 1

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

    def rec_responce(self):
        while self.runing:
            self.lc_r.handle()
            time.sleep(0.002)

    def send_publish(self):
        while self.runing:
            self.send_lock.acquire()

            if self.delay_cnt > 20:
                self.lc_s.publish("robot_control_cmd", self.cmd_msg.encode())
                self.delay_cnt = 0

            self.delay_cnt += 1
            self.send_lock.release()
            time.sleep(0.005)

    def Send_cmd(self, msg):
        self.send_lock.acquire()
        self.delay_cnt = 50
        self.cmd_msg = msg
        self.send_lock.release()

    def Wait_finish(self, mode, gait_id):
        count = 0
        while self.runing and count < 2000: #10s
            if self.mode_ok == mode and self.gait_ok == gait_id:
                return True
            else:
                time.sleep(0.005)
                count += 1

    def quit(self):
        self.runing = 0
        self.rec_thread.join()
        self.send_thread.join()


# ==========================================
# Coordinate transform
# ==========================================
def world_to_robot(dx, dy, yaw):
    fx = math.cos(yaw) * dx + math.sin(yaw) * dy
    fy = -math.sin(yaw) * dx + math.cos(yaw) * dy
    return fx, fy


# ==========================================
# SAFE Gait Transition
# ==========================================
def send_gait(Ctrl, msg, mode, gait, vel, rpy):
    msg.mode = mode
    msg.gait_id = gait
    msg.vel_des = vel
    msg.rpy_des = rpy
    msg.duration = 0
    msg.step_height = [0.03, 0.03]
    msg.life_count += 1

    Ctrl.Send_cmd(msg)

    # 🔥 STABLE TRANSITION
    time.sleep(1.5)


# ==========================================
# MAIN
# ==========================================
def main():
    rclpy.init()

    pose = SimPoseSubscriber()
    threading.Thread(target=lambda: rclpy.spin(pose), daemon=True).start()
    time.sleep(1)

    Ctrl = Robot_Ctrl()
    Ctrl.run()

    msg = robot_control_cmd_lcmt()

    # ==========================================
    # STAND
    # ==========================================
    print("\n[1] STAND")
    send_gait(Ctrl, msg, 12, 0, [0, 0, 0], [0, 0, 0])

    # ==========================================
    # WALK INIT
    # ==========================================
    print("\n[2] WALK gait 26")
    send_gait(Ctrl, msg, 11, 26, [0.12, 0, 0], [0, 0.3, 0])

    # ==========================================
    # WAYPOINTS
    # ==========================================
    WAYPOINTS = [
        (3.0, 7.0, 3.1),
        (2.0, 7.3, 3.1),
        (2.0, 7.3, 1.5),
        
        # 第一组的基准线维持在 x = 1.93
        (1.93, 9.7, 1.5), 
        (1.93, 9.5, 1.5),
        (1.93, 11.2, 1.5),
        (1.93, 10.6, 1.5),
        (1.93, 9.9, 1.5),
        (1.93, 8.8, 1.5),
        (1.93, 8.8, -3.1),

        (1.0, 8.9, -3.1),
        (1.0, 11.0, 1.5),
        (1.0, 11.0, -1.5),
        (1.0, 8.9, -1.5),
        (0.9, 8.8, -3.1),
        
        # 第二组的基准线维持在 x = -0.07
        (-0.07, 9.0, -3.14),   # 修改此处：将 8.8 修改为 9.0
        (-0.07, 9.0, 1.5),     # 修改此处：将 8.8 修改为 9.0
        (-0.07, 9.1, 1.57),    # ptich down
        (-0.07, 10.0, 1.57),
        (-0.07, 11.0, 1.57),
        (-0.07, 10.0, 1.57),
        (-0.07, 10.2, 1.57),   # 📍已修改：定位点3 从 9.8 变为 10.2
        (-0.07, 9.2, 1.57),    # 📍已修改：定位点4 从 9.3 变为 9.2
        (-0.07, 8.2, 1.57),
        
        (0.0, 7.2, -1.57),
        (0.0, 7.2, 0.0),
        (3.0, 7.1, 0.0),
        (3.1, 7.2, 1.5),
    ]

    pid_yaw = PID(2.5, 0.0, 0.1)

    use_custom_gait = False

    # ==========================================
    # LOOP
    # ==========================================
    for gx, gy, gyaw in WAYPOINTS:

        print(f"\n→ Target: {gx:.2f}, {gy:.2f}, {gyaw:.2f}")

        while True:
            x, y, yaw = pose.get()

            dx = gx - x
            dy = gy - y

            fx, fy = world_to_robot(dx, dy, yaw)

            norm = math.hypot(fx, fy) + 1e-6

            vx = 0.12 * fx / norm
            vy = 0.12 * fy / norm
            wz = pid_yaw.update(gyaw - yaw)

            # ==========================================
            # GAIT SELECTION
            # ==========================================
            if use_custom_gait:
                msg.mode = 11
                msg.gait_id = 80    
                msg.vel_des = [2.1*vx, 0.4*vy, 0.4*wz]
                msg.rpy_des = [0, 0.5, 0]
                msg.duration = 0
            else:
                msg.mode = 11
                msg.gait_id = 26
                msg.vel_des = [vx, vy, wz]
                msg.rpy_des = [0, 0, 0]

            msg.life_count = (msg.life_count + 1) % 127
            Ctrl.Send_cmd(msg)
            Ctrl.Wait_finish(msg.mode,msg.gait_id)
            #time.sleep(0.3)

            if math.hypot(dx, dy) < 0.1 :  
                break

            time.sleep(0.05)

        # ==========================================
        # SWITCH LOGIC (4 SWITCHES)
        # ==========================================

        # ==== 第一组 ====
        if abs(gx - 1.93) < 0.01 and abs(gy - 9.5) < 0.01:
            use_custom_gait = True
            print("🔥 SWITCH 1 → gait110 ON (NO FEEDBACK MODE)")

        if abs(gx - 1.93) < 0.01 and abs(gy - 11.2) < 0.01:
            use_custom_gait = False
            print("🔥 SWITCH 2 → gait26 BACK")

        if abs(gx - 1.93) < 0.01 and abs(gy - 10.6) < 0.01:
            use_custom_gait = True
            print("🔥 SWITCH 3 → gait110 ON")

        if abs(gx - 1.93) < 0.01 and abs(gy - 9.9) < 0.01:
            use_custom_gait = False
            print("🔥 SWITCH 4 → gait26 BACK")
        

        # ==== 第二组 ====
        # 修改此处：第二组 Switch 1 触发条件 gy 变更为 9.0
        if abs(gx - (-0.07)) < 0.01 and abs(gy - 9.0) < 0.01:
            use_custom_gait = True
            print("🔥 SWITCH 1 → gait110 ON (NO FEEDBACK MODE)")
            
        if abs(gx - (-0.07)) < 0.01 and abs(gy - 10.0) < 0.01:
            use_custom_gait = False
            print("🔥 SWITCH 2 → gait26 BACK")

        # 📍已修改：同步定位点3的坐标，改为 10.2
        if abs(gx - (-0.07)) < 0.01 and abs(gy - 10.2) < 0.01:
            use_custom_gait = True
            print("🔥 SWITCH 3 → gait110 ON")

        # 📍已修改：同步定位点4的坐标，改为 9.2
        if abs(gx - (-0.07)) < 0.01  and abs(gy - 9.2) < 0.01:
            use_custom_gait = False
            print("🔥 SWITCH 4 → gait26 BACK")
        

    # ==========================================
    # STOP
    # ==========================================
    print("\n✅ FINISHED → DAMPING")

    msg.mode = 7
    msg.gait_id = 0
    msg.life_count += 1
    Ctrl.Send_cmd(msg)

    time.sleep(1.5)

    Ctrl.quit()
    rclpy.shutdown()


if __name__ == "__main__":
    main()