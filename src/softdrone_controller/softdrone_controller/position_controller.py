#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float64MultiArray, Float64, String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import numpy as np
import math

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

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
    def reset(self):
        self.inited = False

class FirstOrderLPF:
    def __init__(self, tau=0.08, init=0.0):
        self.tau, self.y, self.inited = float(tau), float(init), False
    def update(self, x, dt):
        x, dt = float(x), float(max(dt, 1e-4))
        if not self.inited: self.y, self.inited = x, True
        else: self.y = self.y + (dt / (self.tau + dt)) * (x - self.y)
        return self.y
    def reset(self):
        self.inited = False

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
        
        self.declare_parameter("use_ground_truth", False)
        self.use_ground_truth = self.get_parameter("use_ground_truth").value
        
        self.hover_throttle_ratio = float(cfg.HOVER_THROTTLE_RATIO)
        self.vxy_limit = float(cfg.POSITION_VXY_LIMIT)
        self.vz_limit = float(cfg.POSITION_VZ_LIMIT)
        self.max_angle_rate = float(cfg.POSITION_XY_MAX_ANGLE_RATE)
        
        self.current_mode = "MANUAL"
        self.odom_received = False
        
        current_time_sec = self.get_clock().now().nanoseconds / 1e9
        self.last_ctrl_time = current_time_sec
        self.last_debug_print = current_time_sec
        self.last_path_time = current_time_sec
        
        self.pos_w = np.zeros(3); self.vel_w = np.zeros(3); self.yaw_curr = 0.0
        self.roll_curr = 0.0; self.pitch_curr = 0.0 
        self.last_odom_time = 0.0; self.last_pos = np.zeros(3)
        self.first_odom_flag = True 

        self.target_pos_w = np.array([0.0, 0.0, 0.0])
        self.target_yaw = 0.0
        self.is_taking_off = False 
        
        self.path_timeout = 2.0
        self.manual_cmd_cache = Vector3()
        
        self._init_filters_and_pids()
        
        self.laser_z = 0.0; self.laser_received = False
        self.tf_fail_count = 0 
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        if self.use_ground_truth:
            odom_topic = '/rotors/odometry'
            self.get_logger().info(f"📡 模式: [真值控制] | 订阅话题: {odom_topic}")
            self.sub_odom = self.create_subscription(Odometry, odom_topic, self.odom_callback, 10)
        else:
            self.get_logger().warn(f"⚠️ 模式: [2D SLAM 激光控制] | 监听 TF: map -> swift_pico/base_link")
            self.slam_timer = self.create_timer(0.02, self.slam_tf_callback)
            self.last_tf_time = self.get_clock().now().nanoseconds / 1e9

        self.sub_altimeter = self.create_subscription(LaserScan, '/altimeter', self.altimeter_callback, 10)
        self.sub_mode = self.create_subscription(String, '/drone_mode', self.mode_callback, 10)
        self.sub_setpoint = self.create_subscription(Float64MultiArray, '/pos_setpoint', self.setpoint_callback, 10)
        self.sub_manual = self.create_subscription(Vector3, '/manual_attitude_cmd', self.manual_cmd_callback, 10)
        self.pub_att_cmd = self.create_publisher(Vector3, '/attitude_position_cmd', 10)
        self.pub_yaw_cmd = self.create_publisher(Float64, '/yaw_position_cmd', 10)
        
        self.get_logger().info(f"✅ PositionController 启动 | XY轴解耦参数版")

    def _init_filters_and_pids(self):
        self.x_f = AlphaFilter(alpha=cfg.POSITION_FILTER_ALPHA_POS)
        self.y_f = AlphaFilter(alpha=cfg.POSITION_FILTER_ALPHA_POS)
        self.z_f = AlphaFilter(alpha=cfg.POSITION_FILTER_ALPHA_POS)
        self.vx_lpf = FirstOrderLPF(tau=0.15); self.vy_lpf = FirstOrderLPF(tau=0.15); self.vz_lpf = FirstOrderLPF(tau=0.15) 
        self.vx_sp_lpf = FirstOrderLPF(tau=0.04); self.vy_sp_lpf = FirstOrderLPF(tau=0.04); self.vz_sp_lpf = FirstOrderLPF(tau=0.06)
        
        # 💡 [解耦修改]: 分别读取 X 轴和 Y 轴的独立参数
        self.pos_x_pid = PID(cfg.POSITION_X_KP, cfg.POSITION_X_KI, cfg.POSITION_X_KD, cfg.POSITION_X_INT_LIMIT, -cfg.POSITION_X_INT_LIMIT)
        self.pos_y_pid = PID(cfg.POSITION_Y_KP, cfg.POSITION_Y_KI, cfg.POSITION_Y_KD, cfg.POSITION_Y_INT_LIMIT, -cfg.POSITION_Y_INT_LIMIT)
        self.pos_z_pid = PID(cfg.POSITION_Z_KP, cfg.POSITION_Z_KI, cfg.POSITION_Z_KD, cfg.POSITION_Z_INT_LIMIT, -cfg.POSITION_Z_INT_LIMIT)
        
        self.vel_x_pid = PID(cfg.VELOCITY_X_KP, cfg.VELOCITY_X_KI, cfg.VELOCITY_X_KD, cfg.VELOCITY_X_INT_LIMIT, -cfg.VELOCITY_X_INT_LIMIT)
        self.vel_y_pid = PID(cfg.VELOCITY_Y_KP, cfg.VELOCITY_Y_KI, cfg.VELOCITY_Y_KD, cfg.VELOCITY_Y_INT_LIMIT, -cfg.VELOCITY_Y_INT_LIMIT)
        self.vel_z_pid = PID(cfg.VELOCITY_Z_KP, cfg.VELOCITY_Z_KI, cfg.VELOCITY_Z_KD, cfg.VELOCITY_Z_INT_LIMIT, -cfg.VELOCITY_Z_INT_LIMIT)
        
        self.last_roll_cmd = 0.0; self.last_pitch_cmd = 0.0

    def altimeter_callback(self, msg: LaserScan):
        if len(msg.ranges) > 0 and not math.isinf(msg.ranges[0]) and not math.isnan(msg.ranges[0]):
            raw_z = float(msg.ranges[0])
            if 0.001 < raw_z < 10.0:
                self.laser_z = raw_z * math.cos(self.roll_curr) * math.cos(self.pitch_curr)
                self.laser_received = True

    def setpoint_callback(self, msg: Float64MultiArray):
        if len(msg.data) >= 3:
            self.target_pos_w[0], self.target_pos_w[1], self.target_pos_w[2] = msg.data[0], msg.data[1], msg.data[2]
            self.target_yaw = 0.0
            self.last_path_time = self.get_clock().now().nanoseconds / 1e9

    def slam_tf_callback(self):
        if self.use_ground_truth: return 
        try:
            t = self.tf_buffer.lookup_transform('map', 'swift_pico/base_link', rclpy.time.Time())
            self.tf_fail_count = 0 
        except TransformException as ex:
            self.tf_fail_count += 1
            if self.tf_fail_count % 25 == 0: self.get_logger().error("🛑 SLAM TF 获取失败！")
            return

        now_ns = self.get_clock().now().nanoseconds / 1e9
        dt_odom = now_ns - self.last_tf_time
        if dt_odom < 0.005: return
        self.last_tf_time = now_ns

        curr_x, curr_y, curr_z = t.transform.translation.x, t.transform.translation.y, t.transform.translation.z

        if self.first_odom_flag:
            self.last_pos[:] = [curr_x, curr_y, curr_z]
            self.first_odom_flag = False

        self.pos_w[0], self.pos_w[1] = self.x_f.update(curr_x), self.y_f.update(curr_y)
        self.pos_w[2] = self.z_f.update(self.laser_z) if self.laser_received else self.z_f.update(curr_z)

        q = [t.transform.rotation.w, t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z]
        self.yaw_curr = quat_to_yaw(q)
        self.roll_curr, self.pitch_curr, _ = quat_to_eul(q)

        raw_vx = (curr_x - self.last_pos[0]) / dt_odom
        raw_vy = (curr_y - self.last_pos[1]) / dt_odom
        raw_vz = (curr_z - self.last_pos[2]) / dt_odom
        self.last_pos[:] = [curr_x, curr_y, curr_z]

        self.vel_w[0], self.vel_w[1], self.vel_w[2] = self.vx_lpf.update(raw_vx, dt_odom), self.vy_lpf.update(raw_vy, dt_odom), self.vz_lpf.update(raw_vz, dt_odom)
        self.odom_received = True
        self.control_loop()

    def odom_callback(self, msg: Odometry):
        if not self.use_ground_truth: return
        self.odom_received = True
        now_ns = self.get_clock().now().nanoseconds / 1e9
        dt_odom = max(now_ns - self.last_odom_time, 0.01)
        self.last_odom_time = now_ns
        
        curr_x, curr_y, curr_z = msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z
        if self.first_odom_flag:
            self.last_pos[:] = [curr_x, curr_y, curr_z]
            self.first_odom_flag = False

        self.pos_w[0], self.pos_w[1], self.pos_w[2] = self.x_f.update(curr_x), self.y_f.update(curr_y), self.z_f.update(curr_z)
        q = [msg.pose.pose.orientation.w, msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z]
        self.yaw_curr = quat_to_yaw(q)
        self.roll_curr, self.pitch_curr, _ = quat_to_eul(q)
        
        raw_vx = (curr_x - self.last_pos[0]) / dt_odom
        raw_vy = (curr_y - self.last_pos[1]) / dt_odom
        raw_vz = (curr_z - self.last_pos[2]) / dt_odom
        self.last_pos[:] = [curr_x, curr_y, curr_z]

        self.vel_w[0], self.vel_w[1], self.vel_w[2] = self.vx_lpf.update(raw_vx, dt_odom), self.vy_lpf.update(raw_vy, dt_odom), self.vz_lpf.update(raw_vz, dt_odom)
        self.control_loop()

    def mode_callback(self, msg: String):
        if self.current_mode != msg.data:
            self.get_logger().info(f"🔄 模式切换: {self.current_mode} -> {msg.data}")
            self.current_mode = msg.data
            if self.current_mode != "MANUAL":
                self._reset_state() 
                if self.pos_w[2] < 0.5:
                    self.is_taking_off = True
                    self.get_logger().info("🚀 检测到在地面起飞：开启姿态锁定，先上升高度...")
                else:
                    self.is_taking_off = False

    def manual_cmd_callback(self, msg: Vector3):
        self.manual_cmd_cache = msg

    def control_loop(self):
        if not self.odom_received: return
        if self.current_mode == "MANUAL":
            self.pub_att_cmd.publish(self.manual_cmd_cache); return
            
        now = self.get_clock().now().nanoseconds / 1e9
        dt_ctrl = float(np.clip(now - self.last_ctrl_time, 0.01, 0.1))
        self.last_ctrl_time = now
        self.target_yaw = 0.0
        
        path_age = now - self.last_path_time
        if path_age > self.path_timeout:
            if now - self.last_debug_print > 2.0:
                self.get_logger().warn(f"⚠️ 指令超时 ({path_age:.1f}s)，维持最后目标点")
                
        current_target_z = self.target_pos_w[2]
                
        if getattr(self, 'is_taking_off', False):
            if abs(self.pos_w[2] - current_target_z) < 0.2:
                self.is_taking_off = False
                self.get_logger().info("✅ 接近目标高度，解除姿态锁定，允许XY方向平移！")
                
        err_pos = np.clip([self.target_pos_w[0] - self.pos_w[0], 
                           self.target_pos_w[1] - self.pos_w[1], 
                           current_target_z - self.pos_w[2]], -2.0, 2.0)
        
        if abs(err_pos[0]) < 0.02: err_pos[0] = 0.0
        if abs(err_pos[1]) < 0.02: err_pos[1] = 0.0
        
        # 1. 计算期望速度
        v_target_w = np.zeros(3)
        v_target_w[0] = self.pos_x_pid.step(err_pos[0], dt_ctrl)
        v_target_w[1] = self.pos_y_pid.step(err_pos[1], dt_ctrl)
        v_target_w[2] = self.pos_z_pid.step(err_pos[2], dt_ctrl)
        
        # 2. 期望速度滤波限幅
        v_target_w[0] = self.vx_sp_lpf.update(np.clip(v_target_w[0], -self.vxy_limit, self.vxy_limit), dt_ctrl)
        v_target_w[1] = self.vy_sp_lpf.update(np.clip(v_target_w[1], -self.vxy_limit, self.vxy_limit), dt_ctrl)
        v_target_w[2] = self.vz_sp_lpf.update(np.clip(v_target_w[2], -self.vz_limit, self.vz_limit), dt_ctrl)
        
        # 3. 速度坐标系投影
        v_err_w = v_target_w - self.vel_w
        cy, sy = np.cos(self.yaw_curr), np.sin(self.yaw_curr)
        v_err_b_x =  v_err_w[0] * cy + v_err_w[1] * sy
        v_err_b_y = -v_err_w[0] * sy + v_err_w[1] * cy
        
        # 4. 速度PID计算姿态角
        if getattr(self, 'is_taking_off', False):
            pitch_cmd_raw = 0.0 
            roll_cmd_raw = 0.0
            state_str = "TAKEOFF-LOCKED"
        else:
            # 💡 [控制逻辑未变]
            pitch_cmd_raw = self.vel_x_pid.step(v_err_b_x, dt_ctrl)
            roll_cmd_raw  = -self.vel_y_pid.step(v_err_b_y, dt_ctrl)
            state_str = "XY-TRACKING"
            
        SAFE_TILT = 0.4
        roll_cmd_rad = self._slew(np.clip(roll_cmd_raw, -SAFE_TILT, SAFE_TILT), self.last_roll_cmd, dt_ctrl, self.max_angle_rate)
        pitch_cmd_rad = self._slew(np.clip(pitch_cmd_raw, -SAFE_TILT, SAFE_TILT), self.last_pitch_cmd, dt_ctrl, self.max_angle_rate)
        self.last_roll_cmd, self.last_pitch_cmd = roll_cmd_rad, pitch_cmd_rad
        
        # 5. 推力计算
        pid_z_out = self.vel_z_pid.step(v_err_w[2], dt_ctrl)
        base_ratio = self.hover_throttle_ratio + float(np.clip(pid_z_out, -0.4, 0.4))
        c_comp = min(float(np.clip(np.cos(roll_cmd_rad) * np.cos(pitch_cmd_rad), 0.25, 1.0)), float(np.clip(np.cos(self.roll_curr) * np.cos(self.pitch_curr), 0.25, 1.0))) 
        final_throttle_ratio = np.clip(base_ratio * (1.0 + ((1.0 / c_comp) - 1.0) * 1.5), 0.05, 0.95)
        throttle_pwm = 1000.0 + final_throttle_ratio * 1000.0

        self.pub_att_cmd.publish(Vector3(x=float(roll_cmd_rad), y=float(pitch_cmd_rad), z=float(throttle_pwm)))
        self.pub_yaw_cmd.publish(Float64(data=0.0))
        
        if now - self.last_debug_print > 0.5:
            laser_status = "LDR" if self.laser_received else "SLM"
            self.get_logger().info(
                f"[{self.current_mode}-{state_str}] Z({laser_status}):{self.pos_w[2]:.2f}m | TgtZ:{current_target_z:.2f}m | "
                f"Yaw:{np.rad2deg(self.yaw_curr):.1f}° | Thr:{throttle_pwm:.0f}"
            )
            self.last_debug_print = now

    def _slew(self, target, last, dt, max_rate):
        return float(np.clip(target, last - float(max_rate) * float(max(dt, 1e-4)), last + float(max_rate) * float(max(dt, 1e-4))))

    def _reset_state(self):
        self.pos_x_pid.reset(); self.pos_y_pid.reset(); self.pos_z_pid.reset()
        self.vel_x_pid.reset(); self.vel_y_pid.reset(); self.vel_z_pid.reset()
        self.x_f.reset(); self.y_f.reset(); self.z_f.reset()
        self.vx_lpf.reset(); self.vy_lpf.reset(); self.vz_lpf.reset()
        self.vx_sp_lpf.reset(); self.vy_sp_lpf.reset(); self.vz_sp_lpf.reset()
        self.first_odom_flag = True; self.last_roll_cmd = 0.0; self.last_pitch_cmd = 0.0

def main(args=None):
    rclpy.init(args=args)
    node = PositionController()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try: executor.spin()
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()

if __name__ == "__main__": main()