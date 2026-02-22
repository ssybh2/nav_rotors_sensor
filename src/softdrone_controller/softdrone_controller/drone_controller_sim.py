#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
import numpy as np
import threading
import math

from geometry_msgs.msg import Vector3
from std_msgs.msg import String, Float64
from sensor_msgs.msg import Imu          # ← 新增：使用真实IMU
from actuator_msgs.msg import Actuators
from softdrone_controller.config import controller_params as cfg

# ==================== 数学工具 ====================
def wrap_pi(a):
    return float(np.arctan2(np.sin(a), np.cos(a)))

def quat_normalize(q):
    q = np.asarray(q, dtype=float)
    n = np.linalg.norm(q)
    if n < 1e-12: return np.array([1.0, 0.0, 0.0, 0.0])
    return q / n

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

def eul2quat(roll, pitch, yaw):
    cr = np.cos(roll * 0.5); sr = np.sin(roll * 0.5)
    cp = np.cos(pitch * 0.5); sp = np.sin(pitch * 0.5)
    cy = np.cos(yaw * 0.5); sy = np.sin(yaw * 0.5)
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return quat_normalize(np.array([w, x, y, z]))

def calculateErrorQuaternion(q_cmd, q_meas):
    q_meas_conj = np.array([q_meas[0], -q_meas[1], -q_meas[2], -q_meas[3]])
    q_e = np.zeros(4)
    q_e[0] = q_meas_conj[0]*q_cmd[0] - q_meas_conj[1]*q_cmd[1] - q_meas_conj[2]*q_cmd[2] - q_meas_conj[3]*q_cmd[3]
    q_e[1] = q_meas_conj[0]*q_cmd[1] + q_meas_conj[1]*q_cmd[0] + q_meas_conj[2]*q_cmd[3] - q_meas_conj[3]*q_cmd[2]
    q_e[2] = q_meas_conj[0]*q_cmd[2] - q_meas_conj[1]*q_cmd[3] + q_meas_conj[2]*q_cmd[0] + q_meas_conj[3]*q_cmd[1]
    q_e[3] = q_meas_conj[0]*q_cmd[3] + q_meas_conj[1]*q_cmd[2] - q_meas_conj[2]*q_cmd[1] + q_meas_conj[3]*q_cmd[0]
    return q_e

def pwm_to_sim_rads(pwm_val):
    PWM_MIN, PWM_MAX = 1000.0, 2000.0
    MAX_RAD_S = 800.0
    pwm_clipped = np.clip(pwm_val, PWM_MIN, PWM_MAX)
    return float((pwm_clipped - PWM_MIN) * (MAX_RAD_S / (PWM_MAX - PWM_MIN)))

# ==================== PID 类 ====================
class ImprovedPID:
    def __init__(self, kp, ki, kd, i_max=0.5, i_min=-0.5, axis=""):
        self.kp = kp; self.ki = ki; self.kd = kd
        self.i_max = i_max; self.i_min = i_min
        self.axis = axis
        self.integral = 0.0
        self.prev_measurement = 0.0
        self.d_term_sign = -1.0

    def update(self, setpoint, measurement, dt):
        if dt <= 1e-6: dt = 0.001
        error = wrap_pi(setpoint - measurement) if self.axis == "yaw" else (setpoint - measurement)
        p_term = self.kp * error
        self.integral = np.clip(self.integral + error * dt, self.i_min, self.i_max)
        i_term = self.ki * self.integral
        measurement_rate = (measurement - self.prev_measurement) / dt
        d_term = self.kd * measurement_rate * self.d_term_sign
        self.prev_measurement = measurement
        return float(p_term + i_term + d_term)

    def reset(self):
        self.integral = 0.0
        self.prev_measurement = 0.0

# ==================== 主飞控节点 ====================
class DroneControllerSim(Node):
    def __init__(self):
        super().__init__("drone_controller_sim")
        self.CONTROL_FREQ = cfg.CONTROL_FREQ
        self.EXPECTED_DT = 1.0 / self.CONTROL_FREQ
        self.PWM_IDLE = float(cfg.DSHOT_MIN)
        self._init_ros()
        self._init_data()
        self._init_controllers()
        
        current_time_sec = self.get_clock().now().nanoseconds / 1e9
        self.last_loop_time = current_time_sec
        self.loop_count = 0
        self.last_log_time = current_time_sec
        
        self.cmd_log_r = 0.0
        self.cmd_log_p = 0.0
        
        self.control_timer = self.create_timer(self.EXPECTED_DT / 2.0, self._control_loop)
        self.get_logger().info(f"✅ 底层飞控 (增强混控版) 启动 | 已切换为真实IMU模式（零作弊）")

    def _init_ros(self):
        qos_reliable = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE, depth=10)
        
        # ==================== 关键修改 ====================
        # 删除原来的 odom 订阅，改为订阅真实IMU
        self.sub_imu = self.create_subscription(Imu, "/imu", self._imu_callback, qos_reliable)
        
        # 保留其他订阅不变
        self.sub_cmd = self.create_subscription(Vector3, "/attitude_position_cmd", self._cmd_callback, qos_reliable)
        self.sub_yaw = self.create_subscription(Float64, "/yaw_position_cmd", self._yaw_cb, qos_reliable)
        self.sub_arm = self.create_subscription(String, "/drone_arm_cmd", self._arm_callback, qos_reliable)
        
        self.pub_motor = self.create_publisher(Actuators, "/rotors/command/motor_speed", qos_reliable)
        self.lock = threading.Lock()

    def _init_data(self):
        self.state = {"quat": np.array([1.0, 0.0, 0.0, 0.0]), "roll": 0.0, "pitch": 0.0, "yaw": 0.0, "gyro": np.array([0.0, 0.0, 0.0])}
        self.cmd = {"roll": 0.0, "pitch": 0.0, "throttle": self.PWM_IDLE, "yaw_sp": None}
        self.is_armed = False

    def _init_controllers(self):
        self.pid_roll_rate = ImprovedPID(**cfg.PID_ROLL_RATE, axis="roll_rate")
        self.pid_pitch_rate = ImprovedPID(**cfg.PID_PITCH_RATE, axis="pitch_rate")
        self.pid_yaw_rate = ImprovedPID(**cfg.PID_YAW_RATE, axis="yaw_rate")
        self.torque_limit_rp = 0.8 
        self.torque_limit_yaw = 0.5 
        self.yaw_dshot_gain = 0.3

    def _reset_controllers(self):
        self.pid_roll_rate.reset(); self.pid_pitch_rate.reset(); self.pid_yaw_rate.reset()
        self.cmd["yaw_sp"] = None

    # ==================== 核心替换：IMU回调 ====================
    def _imu_callback(self, msg: Imu):
        with self.lock:
            # 从真实IMU直接获取姿态和角速度
            q = [msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z]
            self.state["quat"] = quat_normalize(q)
            self.state["roll"], self.state["pitch"], self.state["yaw"] = quat_to_eul(q)
            self.state["gyro"] = np.array([
                msg.angular_velocity.x,
                msg.angular_velocity.y,
                msg.angular_velocity.z
            ])

    def _cmd_callback(self, msg: Vector3):
        with self.lock:
            self.cmd["roll"], self.cmd["pitch"], self.cmd["throttle"] = msg.x, msg.y, msg.z
            self.cmd_log_r, self.cmd_log_p = msg.x, msg.y

    def _yaw_cb(self, msg: Float64):
        with self.lock: self.cmd["yaw_sp"] = msg.data

    def _arm_callback(self, msg: String):
        with self.lock:
            if msg.data == "TOGGLE":
                self.is_armed = not self.is_armed
                if not self.is_armed:
                    self._reset_controllers()
                    self._publish_motor_speed([self.PWM_IDLE]*4)
                self.get_logger().info(f"⚡ {'已解锁' if self.is_armed else '已上锁'}")

    # ==================== 以下所有代码完全不动 ====================
    def _control_loop(self):
        now = self.get_clock().now().nanoseconds / 1e9
        elapsed = now - self.last_loop_time
        if elapsed < 0: elapsed = self.EXPECTED_DT
        if elapsed < (self.EXPECTED_DT * 0.8): return
        
        real_dt = elapsed
        self.last_loop_time = now
        self.loop_count += 1
        
        if not self.is_armed:
            self._publish_motor_speed([self.PWM_IDLE]*4); return
            
        with self.lock:
            target_r, target_p = self.cmd["roll"], self.cmd["pitch"]
            if self.cmd["yaw_sp"] is None: self.cmd["yaw_sp"] = self.state["yaw"]
            target_y = self.cmd["yaw_sp"]
            
            quat_sp = eul2quat(target_r, target_p, target_y)
            quat_curr = self.state["quat"]
            if np.dot(quat_sp, quat_curr) < 0: quat_sp = -quat_sp
            q_e = calculateErrorQuaternion(quat_sp, quat_curr)
            sign = 1.0 if q_e[0] >= 0 else -1.0
            
            k_angle = 10.0 
            target_rate_roll = sign * q_e[1] * k_angle * cfg.Kp_ANGLE
            target_rate_pitch = sign * q_e[2] * k_angle * cfg.Kp_ANGLE
            target_rate_yaw = sign * q_e[3] * k_angle * cfg.Kp_ANGLE
            
            RATE_LIMIT = 6.0 
            target_rate_roll = np.clip(target_rate_roll, -RATE_LIMIT, RATE_LIMIT)
            target_rate_pitch = np.clip(target_rate_pitch, -RATE_LIMIT, RATE_LIMIT)
            
            gyro = self.state["gyro"]
            torque_roll = self.pid_roll_rate.update(target_rate_roll, gyro[0], real_dt)
            torque_pitch = self.pid_pitch_rate.update(target_rate_pitch, gyro[1], real_dt)
            torque_yaw = self.pid_yaw_rate.update(target_rate_yaw, gyro[2], real_dt)
            
            torque_roll = np.clip(torque_roll, -self.torque_limit_rp, self.torque_limit_rp)
            torque_pitch = np.clip(torque_pitch, -self.torque_limit_rp, self.torque_limit_rp)
            torque_yaw = np.clip(torque_yaw, -self.torque_limit_yaw, self.torque_limit_yaw)
            
            motor_pwm = self._motor_mix(self.cmd["throttle"], torque_roll, torque_pitch, torque_yaw)
            self._publish_motor_speed(motor_pwm)
            
        if now - self.last_log_time > 1.0:
            self.get_logger().info(
                f"HZ:{self.loop_count} | PWM:[{np.min(motor_pwm):.0f} - {np.max(motor_pwm):.0f}] | "
                f"Cmd_Deg(R/P):{np.rad2deg(self.cmd_log_r):.1f}/{np.rad2deg(self.cmd_log_p):.1f} | "
                f"Act_Deg(R/P/Y):{np.rad2deg(self.state['roll']):.1f}/{np.rad2deg(self.state['pitch']):.1f}/{np.rad2deg(self.state['yaw']):.1f}"
            )
            self.last_log_time = now; self.loop_count = 0

    def _motor_mix(self, throttle, tr, tp, ty):
        base = throttle
        ty_gain = ty * self.yaw_dshot_gain
        M = np.array(cfg.MIX_MATRIX)
        scale = cfg.DSHOT_SCALE
        
        combined_torque = (M[:, 0] * tr + M[:, 1] * tp + M[:, 2] * ty_gain) * scale
        
        max_t = np.max(np.abs(combined_torque))
        torque_budget = 350.0 
        if max_t > torque_budget: 
            combined_torque = combined_torque * (torque_budget / max_t)
            
        motors = base + combined_torque
        
        mm_max = np.max(motors)
        if mm_max > 2000.0: 
            motors -= (mm_max - 2000.0)
            
        mm_min = np.min(motors)
        dynamic_min_pwm = max(1350.0, base - 200.0) 
        
        if mm_min < dynamic_min_pwm: 
            motors += (dynamic_min_pwm - mm_min)
            
        return np.clip(motors, cfg.DSHOT_MIN, cfg.DSHOT_MAX)

    def _publish_motor_speed(self, pwms):
        msg = Actuators()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "swift_pico/base_link"
        msg.velocity = [pwm_to_sim_rads(p) for p in pwms]
        self.pub_motor.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = DroneControllerSim()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node(); rclpy.shutdown()

if __name__ == "__main__":
    main()