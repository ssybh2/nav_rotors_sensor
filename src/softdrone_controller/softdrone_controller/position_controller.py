import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float64MultiArray, Float64, String
from nav_msgs.msg import Odometry
import numpy as np
import time
import math
from softdrone_controller.config import controller_params as cfg
# ================= 工具函数 =================
def wrap_pi(a):
    return float(np.arctan2(np.sin(a), np.cos(a)))

def quat_to_yaw(q):
    w, x, y, z = q
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    return np.arctan2(siny_cosp, cosy_cosp)

def quat_to_eul(q):
    w, x, y, z = q
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)
    sinp = 2 * (w * y - z * x)
    pitch = np.copysign(np.pi / 2, sinp) if abs(sinp) >= 1 else np.arcsin(sinp)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)
    return float(roll), float(pitch), float(yaw)

class AlphaFilter:
    def __init__(self, alpha=0.6, init=0.0):
        self.alpha, self.filtered_, self.inited = float(alpha), float(init), False

    def update(self, raw):
        raw = float(raw)
        if not self.inited: self.filtered_, self.inited = raw, True
        else: self.filtered_ = self.alpha * raw + (1.0 - self.alpha) * self.filtered_
        return self.filtered_

class FirstOrderLPF:
    def __init__(self, tau=0.08, init=0.0):
        self.tau, self.y, self.inited = float(tau), float(init), False

    def update(self, x, dt):
        x, dt = float(x), float(max(dt, 1e-4))
        if not self.inited: self.y, self.inited = x, True
        else: self.y = self.y + (dt / (self.tau + dt)) * (x - self.y)
        return self.y

class PID:
    def __init__(self, kp, ki, kd, i_max=0.5, i_min=-0.5):
        self.kp, self.ki, self.kd = float(kp), float(ki), float(kd)
        self.i_max, self.i_min = float(i_max), float(i_min)
        self.integral, self.prev_error = 0.0, None

    def reset(self):
        self.integral, self.prev_error = 0.0, None

    def step(self, error, dt):
        dt = float(np.clip(dt, 1e-4, 0.1))
        self.integral = float(np.clip(self.integral + error * dt, self.i_min, self.i_max))
        d_error = 0.0
        if self.prev_error is not None: d_error = (error - self.prev_error) / dt
        self.prev_error = error
        return self.kp * error + self.ki * self.integral + self.kd * d_error

class PositionController(Node):
    def __init__(self):
        super().__init__('position_control_node')
        # 参数初始化
        self.hover_throttle_ratio = float(cfg.HOVER_THROTTLE_RATIO)
        self.vxy_limit = float(cfg.POSITION_VXY_LIMIT)
        self.vz_limit = float(cfg.POSITION_VZ_LIMIT)
        self.max_climb_rate = float(cfg.TAKEOFF_CLIMB_RATE)
        self.max_angle_rate = float(cfg.POSITION_XY_MAX_ANGLE_RATE)
        self.current_mode = "MANUAL"
        self.hold_state = "TAKEOFF_HOLD"  # 新增：HOLD模式子状态，仿PATH优先Z
        self.odom_received = False
        self.prev_locked = True
        self.last_ctrl_time = time.time()
        self.last_debug_print = time.time()
        self.pos_w = np.zeros(3); self.vel_w = np.zeros(3); self.yaw_curr = 0.0
        self.roll_curr = 0.0  # 新增：实际 roll
        self.pitch_curr = 0.0  # 新增：实际 pitch
        self.last_odom_time = 0.0
        # 目标状态
        self.target_pos_w = np.array([0.0, 0.0, 0.0])
        self.target_yaw = 0.0
        self.height_sp = None
        self.height_target = 0.0
        # 指令超时管理
        self.path_timeout = 2.0 # 增加到2秒，防止仿真卡顿导致误触发
        self.last_path_time = time.time()
        self.has_received_setpoint = False
        self.manual_cmd_cache = Vector3()
        # 滤波器与PID
        self._init_filters_and_pids()
        # ROS 订阅与发布
        self.sub_odom = self.create_subscription(Odometry, '/rotors/odometry', self.odom_callback, 10)
        self.sub_mode = self.create_subscription(String, '/drone_mode', self.mode_callback, 10)
        self.sub_setpoint = self.create_subscription(Float64MultiArray, '/pos_setpoint', self.setpoint_callback, 10)
        self.sub_manual = self.create_subscription(Vector3, '/manual_attitude_cmd', self.manual_cmd_callback, 10)
        self.pub_att_cmd = self.create_publisher(Vector3, '/attitude_position_cmd', 10)
        self.pub_yaw_cmd = self.create_publisher(Float64, '/yaw_position_cmd', 10)
        self.get_logger().info(f"✅ PositionController 优化版启动 | 偏航锁定模式: Yaw=0")

    def _init_filters_and_pids(self):
        self.x_f = AlphaFilter(alpha=cfg.POSITION_FILTER_ALPHA_POS)
        self.y_f = AlphaFilter(alpha=cfg.POSITION_FILTER_ALPHA_POS)
        self.z_f = AlphaFilter(alpha=cfg.POSITION_FILTER_ALPHA_POS)
        self.vx_lpf = FirstOrderLPF(tau=0.06) # 💡 减小tau=0.06，减少速度滤波延迟
        self.vy_lpf = FirstOrderLPF(tau=0.06)
        self.vz_lpf = FirstOrderLPF(tau=0.08) # 💡 减小Z tau
        self.vx_sp_lpf = FirstOrderLPF(tau=0.04) # 💡 减小期望速度滤波tau
        self.vy_sp_lpf = FirstOrderLPF(tau=0.04)
        self.vz_sp_lpf = FirstOrderLPF(tau=0.06)
        self.pos_x_pid = PID(cfg.POSITION_XY_KP, cfg.POSITION_XY_KI, cfg.POSITION_XY_KD, cfg.POSITION_XY_INT_LIMIT, -cfg.POSITION_XY_INT_LIMIT)
        self.pos_y_pid = PID(cfg.POSITION_XY_KP, cfg.POSITION_XY_KI, cfg.POSITION_XY_KD, cfg.POSITION_XY_INT_LIMIT, -cfg.POSITION_XY_INT_LIMIT)
        self.pos_z_pid = PID(cfg.POSITION_Z_KP, cfg.POSITION_Z_KI, cfg.POSITION_Z_KD, cfg.POSITION_Z_INT_LIMIT, -cfg.POSITION_Z_INT_LIMIT)
        self.vel_x_pid = PID(cfg.VELOCITY_XY_KP, cfg.VELOCITY_XY_KI, cfg.VELOCITY_XY_KD, cfg.VELOCITY_XY_INT_LIMIT, -cfg.VELOCITY_XY_INT_LIMIT)
        self.vel_y_pid = PID(cfg.VELOCITY_XY_KP, cfg.VELOCITY_XY_KI, cfg.VELOCITY_XY_KD, cfg.VELOCITY_XY_INT_LIMIT, -cfg.VELOCITY_XY_INT_LIMIT)
        self.vel_z_pid = PID(cfg.VELOCITY_Z_KP, cfg.VELOCITY_Z_KI, cfg.VELOCITY_Z_KD, cfg.VELOCITY_Z_INT_LIMIT, -cfg.VELOCITY_Z_INT_LIMIT)
        self.last_roll_cmd = 0.0; self.last_pitch_cmd = 0.0

    def setpoint_callback(self, msg: Float64MultiArray):
        if len(msg.data) >= 3:
            # 更新位置目标
            self.target_pos_w[0], self.target_pos_w[1], self.target_pos_w[2] = msg.data[0], msg.data[1], msg.data[2]
            # 💡 核心修改：无论外部发什么 Yaw，我们内部在这里强制覆盖为 0
            self.target_yaw = 0.0
            self.has_received_setpoint = True
            self.last_path_time = time.time()
            self.height_target = self.target_pos_w[2]

    def odom_callback(self, msg: Odometry):
        self.odom_received = True
        now_ns = self.get_clock().now().nanoseconds / 1e9
        dt_odom = now_ns - self.last_odom_time
        self.last_odom_time = now_ns
        # 状态更新
        self.pos_w[0] = self.x_f.update(msg.pose.pose.position.x)
        self.pos_w[1] = self.y_f.update(msg.pose.pose.position.y)
        self.pos_w[2] = self.z_f.update(msg.pose.pose.position.z)
        q = [msg.pose.pose.orientation.w, msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z]
        self.yaw_curr = quat_to_yaw(q)
        self.roll_curr, self.pitch_curr, _ = quat_to_eul(q)  # 新增：获取实际倾角
        self.vel_w[0] = self.vx_lpf.update(msg.twist.twist.linear.x, dt_odom)
        self.vel_w[1] = self.vy_lpf.update(msg.twist.twist.linear.y, dt_odom)
        self.vel_w[2] = self.vz_lpf.update(msg.twist.twist.linear.z, dt_odom)
        self.control_loop()

    def mode_callback(self, msg: String):
        prev_mode = self.current_mode
        self.current_mode = msg.data
        if prev_mode != self.current_mode:
            self.get_logger().info(f"🔄 模式切换: {prev_mode} -> {self.current_mode}")
        if self.current_mode != "MANUAL":
            # 切换到自动模式时，平滑高度起始点
            self.height_sp = float(self.pos_w[2])
            self._reset_pids()
            if self.current_mode == "HOLD" and self.pos_w[2] < 0.5:  # 如果地面，进入 TAKEOFF_HOLD
                self.hold_state = "TAKEOFF_HOLD"
            else:
                self.hold_state = "FLY_HOLD"
        if self.current_mode == "MANUAL":
            self.prev_locked = True

    def manual_cmd_callback(self, msg: Vector3):
        self.manual_cmd_cache = msg

    def control_loop(self):
        if not self.odom_received: return
        # 手动模式直接透传
        if self.current_mode == "MANUAL":
            self.pub_att_cmd.publish(self.manual_cmd_cache)
            return
        now = time.time()
        dt_ctrl = float(np.clip(now - self.last_ctrl_time, 0.01, 0.1))
        self.last_ctrl_time = now
        # 💡 核心锁定：在计算开始前，再次确保 target_yaw 为 0
        self.target_yaw = 0.0
        # 超时保护逻辑优化
        path_age = now - self.last_path_time
        if path_age > self.path_timeout:
            # 如果信号丢失，保持最后一次的位置目标，不进行任何坐标瞬移
            # 仅在日志输出警告
            if now - self.last_debug_print > 2.0:
                self.get_logger().warn(f"⚠️ 指令超时 ({path_age:.1f}s)，维持最后目标点")
        # 高度平滑规划
        self._update_height_sp_planA(self.pos_w[2], dt_ctrl)
        # 将平滑后的高度赋给当前解算目标
        current_target_z = self.height_sp if self.height_sp is not None else self.target_pos_w[2]
        # 1. 位置环 PID -> 期望速度
        err_pos = np.array([
            self.target_pos_w[0] - self.pos_w[0],
            self.target_pos_w[1] - self.pos_w[1],
            current_target_z - self.pos_w[2]
        ])
        # XY 死区处理
        if abs(err_pos[0]) < 0.03: err_pos[0] = 0.0
        if abs(err_pos[1]) < 0.03: err_pos[1] = 0.0
        err_pos = np.clip(err_pos, -2.0, 2.0)
        v_target_w = np.zeros(3)
        v_target_w[0] = self.pos_x_pid.step(err_pos[0], dt_ctrl)
        v_target_w[1] = self.pos_y_pid.step(err_pos[1], dt_ctrl)
        v_target_w[2] = self.pos_z_pid.step(err_pos[2], dt_ctrl)
        # 速度限幅与滤波
        v_target_w[0] = self.vx_sp_lpf.update(np.clip(v_target_w[0], -self.vxy_limit, self.vxy_limit), dt_ctrl)
        v_target_w[1] = self.vy_sp_lpf.update(np.clip(v_target_w[1], -self.vxy_limit, self.vxy_limit), dt_ctrl)
        v_target_w[2] = self.vz_sp_lpf.update(np.clip(v_target_w[2], -self.vz_limit, self.vz_limit), dt_ctrl)
        # 2. 速度环 PID -> 期望姿态
        v_err_w = v_target_w - self.vel_w
        cy = np.cos(self.yaw_curr); sy = np.sin(self.yaw_curr)
        v_err_b_x = v_err_w[0] * cy + v_err_w[1] * sy
        v_err_b_y = -v_err_w[0] * sy + v_err_w[1] * cy
        # 新增：HOLD TAKEOFF_HOLD 优先 Z，设 XY 倾角=0
        if self.current_mode == "HOLD" and self.hold_state == "TAKEOFF_HOLD":
            pitch_cmd_raw = 0.0  # 强制无 XY 倾角
            roll_cmd_raw = 0.0
            if abs(self.pos_w[2] - self.target_pos_w[2]) < 0.2:  # Z 到达，切换 FLY_HOLD
                self.hold_state = "FLY_HOLD"
                self.get_logger().info("🚀 HOLD起飞完成，开始XY调整！")
        else:
            pitch_cmd_raw = self.vel_x_pid.step(v_err_b_x, dt_ctrl)
            roll_cmd_raw = -self.vel_y_pid.step(v_err_b_y, dt_ctrl)
        # 姿态限幅与斜率控制（Slew Rate）
        SAFE_TILT = 0.9 # 💡 增加到0.52 rad≈30°，允许更大倾角/加速度
        roll_cmd_rad = self._slew(np.clip(roll_cmd_raw, -SAFE_TILT, SAFE_TILT), self.last_roll_cmd, dt_ctrl, self.max_angle_rate)
        pitch_cmd_rad = self._slew(np.clip(pitch_cmd_raw, -SAFE_TILT, SAFE_TILT), self.last_pitch_cmd, dt_ctrl, self.max_angle_rate)
        self.last_roll_cmd, self.last_pitch_cmd = roll_cmd_rad, pitch_cmd_rad
        # 3. 推力计算 (考虑姿态补偿)
        z_out_vel_corr = float(np.clip(v_err_w[2], -0.2, 0.2))  # 增强：增范围到 ±0.2
        base_ratio = self.hover_throttle_ratio + z_out_vel_corr
        # 前馈补偿（基于期望倾角）
        c_comp_expected = float(np.clip(np.cos(roll_cmd_rad) * np.cos(pitch_cmd_rad), 0.4, 1.0))  # 增强：下限减到0.4
        # 反馈补偿（基于实际倾角）
        c_comp_actual = float(np.clip(np.cos(self.roll_curr) * np.cos(self.pitch_curr), 0.4, 1.0))
        c_comp = (c_comp_expected + c_comp_actual) / 2  # 混合前馈+反馈
        final_throttle_ratio = np.clip(base_ratio / c_comp, 0.05, 0.99)  # 增强：放宽限幅
        throttle_pwm = 1000.0 + final_throttle_ratio * 1000.0
        # 4. 发布指令
        cmd_msg = Vector3()
        cmd_msg.x, cmd_msg.y, cmd_msg.z = float(roll_cmd_rad), float(pitch_cmd_rad), float(throttle_pwm)
        self.pub_att_cmd.publish(cmd_msg)
        yaw_msg = Float64(); yaw_msg.data = 0.0 # 💡 始终发布 0
        self.pub_yaw_cmd.publish(yaw_msg)
        # 日志
        if now - self.last_debug_print > 0.5:
            self.get_logger().info(
                f"[{self.current_mode}] Z:{self.pos_w[2]:.2f}m | TgtZ:{current_target_z:.2f}m | "
                f"Yaw:{np.rad2deg(self.yaw_curr):.1f}° | Thr:{throttle_pwm:.0f}"
            )
            self.last_debug_print = now

    def _update_height_sp_planA(self, current_z, dt_ctrl):
        if self.height_sp is None: self.height_sp = float(current_z)
        step = float(self.max_climb_rate) * float(dt_ctrl)
        if self.height_sp < self.height_target: self.height_sp = min(self.height_sp + step, self.height_target)
        else: self.height_sp = max(self.height_sp - step, self.height_target)

    def _slew(self, target, last, dt, max_rate):
        max_step = float(max_rate) * float(max(dt, 1e-4))
        return float(np.clip(target, last - max_step, last + max_step))

    def _reset_pids(self):
        self.pos_x_pid.reset(); self.pos_y_pid.reset(); self.pos_z_pid.reset()
        self.vel_x_pid.reset(); self.vel_y_pid.reset(); self.vel_z_pid.reset()

def main(args=None):
    rclpy.init(args=args)
    node = PositionController()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()