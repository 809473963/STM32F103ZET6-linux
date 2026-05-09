#!/usr/bin/env python3
"""
wheeltec_keyboard_node — 本地键盘控制，发布 /cmd_vel
=====================================================
可运行在树莓派或通过 SSH 在 PC 上运行（需要 TTY）。

按键:
  u  i  o    左前  前  右前
  j  k  l    左    停  右
  m  ,  .    左后  后  右后
  w / e      加速 / 减速（直接发 /wheeltec/speed_set）
  q / Ctrl+C 退出
"""

import math, select, sys, termios, time, threading, tty

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32

# 速度映射（发布到 /cmd_vel）
#  linear.x 和 angular.z 的值只代表"方向"，
#  实际速度由车体档位决定
_VEL = 0.3   # m/s (象征值，用于触发 serial_node 的阈值)
_ANG = 0.8   # rad/s

KEY_MAP: dict[str, tuple[float, float]] = {
    'i': ( _VEL,  0.0),     # 前
    ',': (-_VEL,  0.0),     # 后
    'j': ( 0.0,  _ANG),     # 左
    'l': ( 0.0, -_ANG),     # 右
    'u': ( _VEL,  _ANG),    # 左前
    'o': ( _VEL, -_ANG),    # 右前
    'm': (-_VEL,  _ANG),    # 左后
    '.': (-_VEL, -_ANG),    # 右后
    'k': ( 0.0,  0.0),      # 停
}

BANNER = """\033[1;36m
╔══════════════════════════════════════════╗
║    WHEELTEC 键盘控制  (wheeltec_keyboard) ║
╠══════════════════════════════════════════╣
║   u  i  o    左前  前  右前              ║
║   j  k  l    左    停  右                ║
║   m  ,  .    左后  后  右后              ║
║   w=加速  e=减速  q=退出                 ║
╚══════════════════════════════════════════╝\033[0m
"""


class WheeltecKeyboardNode(Node):
    def __init__(self):
        super().__init__("wheeltec_keyboard_node")
        self._pub_vel   = self.create_publisher(Twist, "/cmd_vel", 10)
        self._pub_speed = self.create_publisher(Int32, "/wheeltec/speed_set", 10)
        self._speed_level = 5
        self.running = True

    def _publish(self, v: float, w: float):
        msg = Twist()
        msg.linear.x  = v
        msg.angular.z = w
        self._pub_vel.publish(msg)

    def _set_speed(self, delta: int):
        self._speed_level = max(1, min(10, self._speed_level + delta))
        msg = Int32()
        msg.data = self._speed_level
        self._pub_speed.publish(msg)
        print(f"\r速度档 → {self._speed_level}   ", flush=True)

    def keyboard_loop(self):
        if not sys.stdin.isatty():
            self.get_logger().error("需要交互式终端！请直接运行，不要通过管道重定向。")
            return

        settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        print(BANNER, flush=True)
        try:
            while rclpy.ok() and self.running:
                readable, _, _ = select.select([sys.stdin], [], [], 0.1)
                if not readable:
                    continue
                key = sys.stdin.read(1)
                if key == 'q':
                    self.running = False
                    self._publish(0.0, 0.0)
                    print("\n\033[1;31m[退出]\033[0m")
                    break
                elif key == 'w':
                    self._set_speed(+1)
                elif key == 'e':
                    self._set_speed(-1)
                elif key in KEY_MAP:
                    v, w = KEY_MAP[key]
                    direction = {
                        'i':'前进','u':'左前','o':'右前',
                        'j':'左转','k':'停止','l':'右转',
                        'm':'左后',',':'后退','.':'右后',
                    }.get(key, '?')
                    print(f"\r  [{direction}]  v={v:+.1f}  w={w:+.1f}   ", flush=True)
                    self._publish(v, w)
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


def main(args=None):
    rclpy.init(args=args)
    node = WheeltecKeyboardNode()
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    try:
        node.keyboard_loop()
    except KeyboardInterrupt:
        pass
    finally:
        node.running = False
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass
        spin_thread.join(timeout=1.0)
