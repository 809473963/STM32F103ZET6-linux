import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import can  # 基于你 6.6.114.1 内核打通的驱动
import struct
import time

class ChassisLogic(Node):
    def __init__(self):
        super().__init__('chassis_logic')
        
        # --- 1. 参数配置 ---
        self.d = 0.5           # 履带间距 (m)
        self.r = 0.05          # 驱动轮半径 (m)
        self.last_cmd_time = self.get_clock().now() # 记录最后一次收到指令的时间
        
        # --- 2. 核心通信设置 ---
        # 订阅遥控器或算法发出的指令流
        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)
        
        # 初始化 SocketCAN (物理连接点)
        try:
            # 调试阶段用 vcan0，实机切换到 can0
            self.bus = can.interface.Bus(channel='vcan0', bustype='socketcan')
            self.get_logger().info('CAN 总线已连接！')
        except Exception as e:
            self.get_logger().error(f'CAN 连接失败: {e}')

        # --- 3. 安全卫士：死人开关 (Deadman Switch) ---
        # 每 0.1 秒检查一次指令是否过期，防止由于遥控器断开导致的“疯跑”
        self.timer = self.create_timer(0.1, self.watchdog_check)
        
        self.get_logger().info('控制节点已就绪！')

    def cmd_callback(self, msg):
        """处理所有方向的移动指令"""
        self.last_cmd_time = self.get_clock().now()
        
        v = msg.linear.x   # 线速度：正为前，负为后
        w = msg.angular.z  # 角速度：正为左转/逆时针，负为右转/顺时针
        
        # 1. 运动学解算 (差速模型)
        # 前进/后退/左转/右转/原地旋转全部通过这个公式涵盖
        v_L = v - (w * self.d / 2)
        v_R = v + (w * self.d / 2)
        
        # 2. 物理单位换算：m/s 转换为电机需要的 rad/s
        # 公式：速度 = 角速度 * 半径 -> 角速度 = 速度 / 半径
        rad_L = v_L / self.r
        rad_R = v_R / self.r
        
        # 3. 执行下发
        self.send_motor_data(0x01, rad_L) # 假设左电机 ID 为 1
        self.send_motor_data(0x02, rad_R) # 假设右电机 ID 为 2
        
        # 调试日志
        self.get_logger().info(f'控制信号 -> 线速:{v:.2f}, 角速:{w:.2f} | 电机信号 -> 左:{rad_L:.2f}, 右:{rad_R:.2f}')

    def send_motor_data(self, motor_id, speed):
        """将计算出的速度打包成达妙 DM-H3510 的 CAN 报文"""
        # 达妙电机速度模式报文通常为 8 字节，前 4 字节为 IEEE 754 浮点数
        try:
            # 使用 struct 将 float 转换为 4 字节小端序二进制
            # 这里的协议细节需要根据你手里的达妙电机手册微调
            data = struct.pack('<f', float(speed)) + b'\x00\x00\x00\x00'
            msg = can.Message(arbitration_id=motor_id, data=data, is_extended_id=False)
            self.bus.send(msg)
        except Exception as e:
            self.get_logger().error(f'CAN 发送异常: {e}')

    def watchdog_check(self):
        """安全保护：如果 0.5 秒没收到指令，强制停机"""
        now = self.get_clock().now()
        duration = (now - self.last_cmd_time).nanoseconds / 1e9
        
        if duration > 0.5:
            # 停止报文：给左右电机发 0 速度
            self.send_motor_data(0x01, 0.0)
            self.send_motor_data(0x02, 0.0)
            # 降低日志频率，避免刷屏
            # self.get_logger().warn('指令超时，底盘已紧急制动！')

def main(args=None):
    rclpy.init(args=args)
    node = ChassisLogic()
    rclpy.spin(node)
    rclpy.shutdown()