#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from std_msgs.msg import String
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import sys
import select
import termios
import tty
import os

# ==========================================
# 👇 用户自定义配置区 (在这里修改你的参数) 👇
# ==========================================

# 1. 基础油门设置 (PWM 1000-2000)
IDLE_THROTTLE = 1000.0       # 怠速/解锁后的初始油门
HOVER_THROTTLE = 1850.0      # [前馈] 悬停油门 (按下 H 键直接跳到此值)

# 2. 灵敏度/步长设置
STEP_THROTTLE = 2.0   # 油门步长 (按一下箭头键增加/减少多少 PWM)
STEP_ROLL_PITCH = 0.05       # 姿态步长 (按一下WASD改变多少弧度, 0.05 rad ≈ 3度)

# 3. 限制设置
MAX_ATTITUDE = 0.6           # 最大倾角限制 (弧度)
MAX_THROTTLE = 2000.0        # 最大油门
MIN_THROTTLE = 1000.0        # 最小油门

# 4. 发送频率
PUBLISH_RATE = 20.0          # Hz

# ==========================================

HELP_MSG = """
==========================================================
   🚁  DRONE COMMANDER PRO (SIMULATION)  🚁
==========================================================
[系统控制]
  R         : 解锁 / 上锁 (Toggle ARM) -> 初始油门重置为 {:.0f}
  Space     : ⚠️  紧急停机 (怠速 & 姿态归零)
  Ctrl+C    : 退出程序

[模式切换]
  M : MANUAL   (手动姿态)
  B : POSITION (定点悬停)
  N : PATH     (画圆轨迹)

[手动飞行控制 (MANUAL模式)]
  H     : 🔥 一键悬停油门 (跳至 {:.0f})
  ↑ / ↓ : 油门 +/- {:.0f}
  
  W / S : Pitch +/- (低头前进 / 抬头后退) [步长: {:.2f}]
  A / D : Roll  -/+ (左倾左移 / 右倾右移) [步长: {:.2f}]
  X     : 姿态回中 (水平)

==========================================================
""".format(IDLE_THROTTLE, HOVER_THROTTLE, STEP_THROTTLE, STEP_ROLL_PITCH, STEP_ROLL_PITCH)

class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')
        qos = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE, depth=10)

        # 发布者
        self.pub_manual_att = self.create_publisher(Vector3, '/manual_attitude_cmd', qos)
        self.pub_mode = self.create_publisher(String, '/drone_mode', qos)
        self.pub_arm = self.create_publisher(String, '/drone_arm_cmd', qos)
        
        # 内部状态
        self.target_roll = 0.0
        self.target_pitch = 0.0
        self.target_throttle = IDLE_THROTTLE
        
        self.current_mode = "MANUAL"
        self.is_armed = False
        self.is_running = True

        # 定时器
        self.timer = self.create_timer(1.0 / PUBLISH_RATE, self._timer_callback)
        
        # 界面初始化
        self._print_interface()

    def _timer_callback(self):
        # 持续发送指令
        msg = Vector3()
        msg.x = float(self.target_roll)
        msg.y = float(self.target_pitch)
        msg.z = float(self.target_throttle)
        self.pub_manual_att.publish(msg)

        # 刷新底部状态栏
        self._print_status_line()

    def _print_interface(self):
        """打印静态帮助信息"""
        os.system('clear')
        print(HELP_MSG)
        print("\033[s") # 保存光标位置

    def _print_status_line(self):
        """动态刷新状态"""
        # 颜色代码
        c_reset = "\033[0m"
        c_red = "\033[91m"
        c_green = "\033[92m"
        c_yellow = "\033[93m"
        c_cyan = "\033[96m"

        state_str = f"{c_green}ARMED {c_reset}" if self.is_armed else f"{c_red}LOCKED{c_reset}"
        mode_str = f"{c_yellow}{self.current_mode:<8s}{c_reset}" if self.current_mode == "MANUAL" else f"{c_cyan}{self.current_mode:<8s}{c_reset}"
        
        # 恢复光标并清除行
        print(f"\033[u\033[K", end="") 
        print(f"[{state_str}] Mode:[{mode_str}] "
              f"Thr:{self.target_throttle:.0f} | "
              f"R:{self.target_roll:.2f} P:{self.target_pitch:.2f}", end="", flush=True)

    def get_key(self):
        settings = termios.tcgetattr(sys.stdin)
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        key = ''
        if rlist:
            key = sys.stdin.read(1)
            if key == '\x1b': 
                key += sys.stdin.read(2)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

    def run(self):
        try:
            while rclpy.ok() and self.is_running:
                key = self.get_key()
                
                # 无按键时处理ROS回调
                if key == '':
                    rclpy.spin_once(self, timeout_sec=0)
                    continue

                # --- 退出 ---
                if key == '\x03': # Ctrl+C
                    self.is_running = False
                    break

                # --- 系统控制 ---
                if key.lower() == 'r':
                    self.is_armed = not self.is_armed
                    self.pub_arm.publish(String(data="TOGGLE"))
                    
                    # 上锁时重置
                    if not self.is_armed:
                        self.target_throttle = IDLE_THROTTLE
                        self.target_roll = 0.0
                        self.target_pitch = 0.0
                    else:
                        # 解锁时，也可以选择自动设为 IDLE
                        self.target_throttle = IDLE_THROTTLE

                elif key.lower() == 'm':
                    self.current_mode = "MANUAL"
                    self.pub_mode.publish(String(data="MANUAL"))
                elif key.lower() == 'b':
                    self.current_mode = "POSITION"
                    self.pub_mode.publish(String(data="HOLD"))
                elif key.lower() == 'n':
                    self.current_mode = "PATH"
                    self.pub_mode.publish(String(data="PATH"))
                
                # --- 飞行指令 (仅修改变量) ---
                
                # 1. 油门控制
                if key == '\x1b[A' or key.lower() == 'i': # 上箭头 / I
                    self.target_throttle = min(self.target_throttle + STEP_THROTTLE, MAX_THROTTLE)
                
                elif key == '\x1b[B' or key.lower() == 'k': # 下箭头 / K
                    self.target_throttle = max(self.target_throttle - STEP_THROTTLE, MIN_THROTTLE)
                
                elif key.lower() == 'h': # [新增] 一键悬停油门
                    self.target_throttle = HOVER_THROTTLE

                # 2. 姿态控制 (FLU: Pitch+低头, Roll+右倾)
                # W 前进 -> Pitch 增加 (+)
                if key.lower() == 'w': 
                    self.target_pitch = min(self.target_pitch + STEP_ROLL_PITCH, MAX_ATTITUDE)
                # S 后退 -> Pitch 减小 (-)
                elif key.lower() == 's': 
                    self.target_pitch = max(self.target_pitch - STEP_ROLL_PITCH, -MAX_ATTITUDE)
                
                # A 左移 -> Roll 减小 (-)
                if key.lower() == 'a': 
                    self.target_roll = max(self.target_roll - STEP_ROLL_PITCH, -MAX_ATTITUDE)
                # D 右移 -> Roll 增加 (+)
                elif key.lower() == 'd': 
                    self.target_roll = min(self.target_roll + STEP_ROLL_PITCH, MAX_ATTITUDE)
                
                # 3. 姿态复位
                if key.lower() == 'x':
                    self.target_roll = 0.0
                    self.target_pitch = 0.0
                
                # 4. 紧急复位 (空格)
                if key == ' ':
                    self.target_throttle = IDLE_THROTTLE
                    self.target_roll = 0.0
                    self.target_pitch = 0.0

                rclpy.spin_once(self, timeout_sec=0)
                
        except Exception as e:
            print(f"\nError: {e}")
        finally:
            print("\nExiting...")

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleop()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()