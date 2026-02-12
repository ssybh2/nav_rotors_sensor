#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Position Cmd 节点（连续轨迹追踪版 - Time-based Trajectory Tracking）
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64MultiArray
from nav_msgs.msg import Odometry
import math
import numpy as np
import time

def wrap_pi(a: float) -> float:
    return float(np.arctan2(np.sin(a), np.cos(a)))

class PositionCmdNode(Node):
    def __init__(self):
        super().__init__("position_cmd_node")
        self.hold_point = [1.0, 1.0, 4.0, 0.0]  # [x, y, z, yaw]
        self.path_config = {
            "center": [0.0, 0.0],
            "radius": 1.5,
            "target_laps": 2,
            "fixed_z": 2.0,
            "fixed_yaw": 0.0,
            "takeoff_height": 2.0,
            "error_tol_z_takeoff": 0.2,
            "waypoint_reach_tol": 0.1,
            "land_error_tol_z": 0.05,
            "cmd_rate_hz": 50.0,
            "log_interval": 0.2,
            "target_speed": 0.15,  # 新增：目标移动速度 (m/s)
        }
        # 计算角速度和总角度
        self.omega = self.path_config["target_speed"] / self.path_config["radius"]  # rad/s
        self.total_angle = 2 * math.pi * self.path_config["target_laps"]
        # 计算起点
        self.start_x = self.path_config["center"][0] + self.path_config["radius"] * math.cos(0)
        self.start_y = self.path_config["center"][1] + self.path_config["radius"] * math.sin(0)
        self.get_logger().info(f"圆起点: ({self.start_x:.2f}, {self.start_y:.2f}, {self.path_config['fixed_z']:.2f})")

        self.sub_mode = self.create_subscription(String, "/drone_mode", self._mode_cb, 10)
        self.sub_odom = self.create_subscription(Odometry, "/rotors/odometry", self._odom_cb, 10)
        self.pub_setpoint = self.create_publisher(Float64MultiArray, "/pos_setpoint", 10)
        self.current_mode = "MANUAL"
        self.real_pos_w = np.zeros(3)
        self.real_yaw = 0.0
        self.path_state = "TAKEOFF"
        self.current_target_point = None
        self.fly_start_time = 0.0  # 新增：飞行开始时间
        self.last_log_time = time.time()
        self.last_publish_z = 0.0
        # ========== 新增：频率统计变量 ==========
        self.cmd_pub_count = 0  # 目标点发布次数
        self.last_pub_time = time.time()  # 上一次发布时间
        self.pub_interval_history = []  # 发布间隔历史
        self.last_freq_log_time = time.time()  # 频率日志上次输出时间
        self.timer = self.create_timer(1.0 / self.path_config["cmd_rate_hz"], self._timer_cb)
        self.get_logger().info("✅ PositionCmdNode (连续轨迹追踪版) 启动完成 | 配置发布频率: {:.1f}Hz".format(self.path_config["cmd_rate_hz"]))

    def _mode_cb(self, msg):
        new_mode = msg.data.upper()
        if new_mode == self.current_mode:
            return
        self.current_mode = new_mode
        self.get_logger().info(f"🔄 切换到 {self.current_mode} 模式")
        if self.current_mode == "PATH":
            self.path_state = "TAKEOFF"
            self.fly_start_time = 0.0
            self.current_target_point = [self.start_x, self.start_y, self.path_config["takeoff_height"], self.path_config["fixed_yaw"]]
            self.get_logger().info(f"📌 PATH模式启动：准备起飞")
        elif self.current_mode == "HOLD":
            self.current_target_point = self.hold_point.copy()

    def _odom_cb(self, msg: Odometry):
        self.real_pos_w[0] = msg.pose.pose.position.x
        self.real_pos_w[1] = msg.pose.pose.position.y
        self.real_pos_w[2] = msg.pose.pose.position.z
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.real_yaw = math.atan2(siny_cosp, cosy_cosp)

    def _print_detailed_log(self):
        now = time.time()
        if now - self.last_log_time < self.path_config["log_interval"]:
            return
        if self.current_target_point is None:
            return
        real_x, real_y, real_z = self.real_pos_w
        target_x, target_y, target_z, target_yaw = self.current_target_point
        xy_error = math.hypot(real_x - target_x, real_y - target_y)
        z_error = abs(real_z - target_z)
        log_prefix = f"📊 【{self.current_mode}/{self.path_state}】"
        log_core = (
            f"位置:({real_x:.2f},{real_y:.2f},{real_z:.2f}) -> 目标:({target_x:.2f},{target_y:.2f},{target_z:.2f}) | "
            f"XY误差:{xy_error:.2f}m | Z误差:{z_error:.2f}m"
        )
        if self.current_mode == "PATH" and self.path_state == "FLY_POINTS":
            elapsed = time.time() - self.fly_start_time
            theta = self.omega * elapsed
            lap_progress = theta / (2 * math.pi)
            log_core += f" | 进度:{lap_progress:.2f} 圈"
        self.get_logger().info(log_prefix + log_core)
        self.last_log_time = now

    # ========== 新增：强化发布函数的日志 ==========
    def _publish_setpoint(self, x, y, z, yaw):
        now = time.time()
        # 计算发布间隔
        pub_interval = now - self.last_pub_time if self.last_pub_time > 0 else 0.0
        self.pub_interval_history.append(pub_interval)
        # 只记录最近100个间隔（避免内存占用）
        if len(self.pub_interval_history) > 100:
            self.pub_interval_history.pop(0)
        # 计算实际发布频率
        avg_interval = np.mean(self.pub_interval_history) if self.pub_interval_history else 0.0
        real_pub_freq = 1.0 / avg_interval if avg_interval > 0 else 0.0
        # 发布消息
        msg = Float64MultiArray()
        msg.data = [float(x), float(y), float(z), float(yaw)]
        self.pub_setpoint.publish(msg)
        self.last_publish_z = z
        self.cmd_pub_count += 1
        self.last_pub_time = now
        # 每1秒输出一次频率统计（避免刷屏）
        if now - self.last_freq_log_time > 1.0:
            self.get_logger().info(
                f"📡 PositionCmd 发布统计 | "
                f"配置频率:{self.path_config['cmd_rate_hz']:.1f}Hz | "
                f"实际平均频率:{real_pub_freq:.1f}Hz | "
                f"平均间隔:{avg_interval*1000:.1f}ms | "
                f"本次发布目标点:({x:.2f},{y:.2f},{z:.2f}) | "
                f"累计发布次数:{self.cmd_pub_count}"
            )
            self.last_freq_log_time = now

    def _timer_cb(self):
        # ========== 新增：定时器回调频率监控 ==========
        timer_cb_start = time.time()
        if self.current_mode == "MANUAL":
            return
        if self.current_mode == "HOLD":
            self._publish_setpoint(self.hold_point[0], self.hold_point[1], self.hold_point[2], self.hold_point[3])
            self.current_target_point = self.hold_point
            self._print_detailed_log()
            return
        if self.current_mode == "PATH":
            if self.path_state == "TAKEOFF":
                target_z = self.path_config["takeoff_height"]
                self._publish_setpoint(self.start_x, self.start_y, target_z, self.path_config["fixed_yaw"])
                self.current_target_point = [self.start_x, self.start_y, target_z, self.path_config["fixed_yaw"]]
                if abs(self.real_pos_w[2] - target_z) < self.path_config["error_tol_z_takeoff"]:
                    self.path_state = "FLY_POINTS"
                    self.fly_start_time = time.time()
                    self.get_logger().info("🚀 起飞完成，开始画圆！")
            elif self.path_state == "FLY_POINTS":
                elapsed = time.time() - self.fly_start_time
                theta = self.omega * elapsed
                if theta >= self.total_angle:
                    self.path_state = "RETURN"
                    self.get_logger().info("🎉 所有圈数执行完毕，准备返航")
                    return
                x = self.path_config["center"][0] + self.path_config["radius"] * math.cos(theta)
                y = self.path_config["center"][1] + self.path_config["radius"] * math.sin(theta)
                z = self.path_config["fixed_z"]
                yaw = self.path_config["fixed_yaw"]
                self._publish_setpoint(x, y, z, yaw)
                self.current_target_point = [x, y, z, yaw]
            elif self.path_state == "RETURN":
                target_z = self.path_config["fixed_z"]
                self._publish_setpoint(self.hold_point[0], self.hold_point[1], target_z, self.hold_point[3])
                self.current_target_point = [self.hold_point[0], self.hold_point[1], target_z, self.hold_point[3]]
                xy_error = math.hypot(self.real_pos_w[0] - self.hold_point[0], self.real_pos_w[1] - self.hold_point[1])
                if xy_error < self.path_config["waypoint_reach_tol"]:
                    self.path_state = "LAND"
                    self.get_logger().info("🛬 到达返航点，开始降落")
            elif self.path_state == "LAND":
                self._publish_setpoint(self.hold_point[0], self.hold_point[1], 0.0, self.hold_point[3])
                self.current_target_point = [self.hold_point[0], self.hold_point[1], 0.0, self.hold_point[3]]
                if self.real_pos_w[2] < 0.19:
                    self.path_state = "DONE"
                    self.get_logger().info("✅ 降落完成")
            elif self.path_state == "DONE":
                self._publish_setpoint(self.hold_point[0], self.hold_point[1], -0.1, self.hold_point[3])
            self._print_detailed_log()
        # ========== 新增：定时器回调耗时监控 ==========
        cb_elapsed = (time.time() - timer_cb_start) * 1000  # 毫秒
        if cb_elapsed > 5.0:  # 回调耗时超过5ms时告警（卡壳特征）
            self.get_logger().warn(
                f"⚠️ PositionCmd 定时器回调耗时过长 | "
                f"耗时:{cb_elapsed:.1f}ms | "
                f"配置周期:{1000/self.path_config['cmd_rate_hz']:.1f}ms"
            )

def main(args=None):
    rclpy.init(args=args)
    rclpy.logging.set_logger_level("position_cmd_node", rclpy.logging.LoggingSeverity.INFO)
    node = PositionCmdNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 PositionCmdNode 手动退出")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()