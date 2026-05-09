#!/usr/bin/env python3
"""
wheeltec_serial_node — 将 /cmd_vel 转换为 WHEELTEC 单字节串口协议
======================================================
WHEELTEC C10B STM32 主板通信协议（与蓝牙/WiFi 完全相同，走串口）：
  0x41  前进       0x42  右前     0x43  右转
  0x44  右后       0x45  后退     0x46  左后
  0x47  左转       0x48  左前
  0x58  加速       0x59  减速
  0x5A  停止
连接方式:
  树莓派 USB-A  ──USB 线──  车体 USB 口 (CH9102 USB-Serial)
  默认设备: /dev/ttyUSB0，115200 波特率
"""

import serial
import threading
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32

# ─── 协议字节 ──────────────────────────────────────────────────────────────────
CMD_FWD        = 0x41
CMD_FWD_RIGHT  = 0x42
CMD_RIGHT      = 0x43
CMD_BWD_RIGHT  = 0x44
CMD_BWD        = 0x45
CMD_BWD_LEFT   = 0x46
CMD_LEFT       = 0x47
CMD_FWD_LEFT   = 0x48
CMD_SPEED_UP   = 0x58
CMD_SPEED_DOWN = 0x59
CMD_STOP       = 0x5A

# 速度档位范围 (WHEELTEC 支持 1~10 档，开机默认 5)
SPEED_MIN = 1
SPEED_MAX = 10
SPEED_DEFAULT = 5


class WheeltecSerialNode(Node):
    def __init__(self):
        super().__init__("wheeltec_serial_node")

        # 参数
        self.declare_parameter("port",        "/dev/ttyUSB0")
        self.declare_parameter("baud",        115200)
        self.declare_parameter("lin_thresh",  0.05)   # m/s 以下视为零
        self.declare_parameter("ang_thresh",  0.1)    # rad/s 以下视为零
        self.declare_parameter("watchdog_s",  0.5)    # 无指令超时后停车(s)

        self._port      = self.get_parameter("port").value
        self._baud      = self.get_parameter("baud").value
        self._lin_thr   = self.get_parameter("lin_thresh").value
        self._ang_thr   = self.get_parameter("ang_thresh").value
        self._watchdog  = self.get_parameter("watchdog_s").value

        # 串口
        self._ser = None
        self._open_serial()

        # 状态
        self._last_cmd_time = time.time()
        self._speed_level   = SPEED_DEFAULT
        self._last_dir_cmd  = CMD_STOP

        # 订阅 /cmd_vel
        self.create_subscription(Twist, "/cmd_vel", self._cmd_vel_cb, 10)

        # 订阅 /wheeltec/speed_set （直接设置档位，范围 1~10）
        self.create_subscription(Int32, "/wheeltec/speed_set", self._speed_set_cb, 10)

        # 看门狗定时器
        self.create_timer(0.1, self._watchdog_cb)

        self.get_logger().info(
            f"WheeltecSerialNode 已启动  port={self._port}  baud={self._baud}")

    # ── 串口 ────────────────────────────────────────────────────────────────────

    def _open_serial(self):
        try:
            self._ser = serial.Serial(
                self._port, self._baud,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.1
            )
            self.get_logger().info(f"串口已打开: {self._port} @ {self._baud}")
        except Exception as e:
            self.get_logger().error(f"串口打开失败: {e}\n请检查 USB 线 / 设备权限 (sudo usermod -aG dialout $USER)")
            self._ser = None

    def _send(self, byte: int):
        if self._ser is None or not self._ser.is_open:
            return
        try:
            self._ser.write(bytes([byte]))
        except Exception as e:
            self.get_logger().warn(f"串口发送失败: {e}")

    # ── cmd_vel → 方向字节 ────────────────────────────────────────────────────

    @staticmethod
    def _vel_to_cmd(v: float, w: float, lin_thr: float, ang_thr: float) -> int:
        fwd  = v >  lin_thr
        bwd  = v < -lin_thr
        lft  = w >  ang_thr
        rgt  = w < -ang_thr

        if   fwd  and lft : return CMD_FWD_LEFT
        elif fwd  and rgt : return CMD_FWD_RIGHT
        elif fwd          : return CMD_FWD
        elif bwd  and lft : return CMD_BWD_LEFT
        elif bwd  and rgt : return CMD_BWD_RIGHT
        elif bwd          : return CMD_BWD
        elif lft          : return CMD_LEFT
        elif rgt          : return CMD_RIGHT
        else              : return CMD_STOP

    def _cmd_vel_cb(self, msg: Twist):
        self._last_cmd_time = time.time()
        v = msg.linear.x
        w = msg.angular.z
        cmd = self._vel_to_cmd(v, w, self._lin_thr, self._ang_thr)
        self._last_dir_cmd = cmd
        self._send(cmd)

    # ── 速度档位控制 ────────────────────────────────────────────────────────────

    def _speed_set_cb(self, msg: Int32):
        """外部直接设定速度档 1-10"""
        target = max(SPEED_MIN, min(SPEED_MAX, int(msg.data)))
        delta = target - self._speed_level
        cmd = CMD_SPEED_UP if delta > 0 else CMD_SPEED_DOWN
        for _ in range(abs(delta)):
            self._send(cmd)
            time.sleep(0.05)
        self._speed_level = target
        self.get_logger().info(f"速度档 → {self._speed_level}")

    # ── 看门狗 ──────────────────────────────────────────────────────────────────

    def _watchdog_cb(self):
        if time.time() - self._last_cmd_time > self._watchdog:
            if self._last_dir_cmd != CMD_STOP:
                self._send(CMD_STOP)
                self._last_dir_cmd = CMD_STOP

    def destroy_node(self):
        self._send(CMD_STOP)
        if self._ser and self._ser.is_open:
            self._ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WheeltecSerialNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
