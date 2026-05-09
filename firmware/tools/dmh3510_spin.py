#!/usr/bin/env python3
"""
DMH3510 电机最简定速旋转
用法: python3 dmh3510_spin.py [--port /dev/ttyUSB0] [--speed 5.0] [--id 2]
依赖: pip install python-can
"""
import struct, time, sys, argparse

try:
    import can
except ImportError:
    print("请先安装: pip install python-can"); sys.exit(1)


def special(bus, mid, cmd):
    """特殊命令: 0xFC=使能  0xFD=失能  0xFB=清错"""
    bus.send(can.Message(arbitration_id=mid,
                         data=[0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, cmd],
                         is_extended_id=False))
    time.sleep(0.05)


def write_reg(bus, mid, rid, u32_val):
    """写寄存器 (uint32, 发到 0x7FF)"""
    d = [mid & 0xFF, (mid >> 8) & 0xFF, 0x55, rid,
         u32_val & 0xFF, (u32_val >> 8) & 0xFF,
         (u32_val >> 16) & 0xFF, (u32_val >> 24) & 0xFF]
    bus.send(can.Message(arbitration_id=0x7FF, data=d, is_extended_id=False))
    time.sleep(0.05)


def set_vel(bus, mid, spd_rad_s):
    """VEL 模式速度指令, CAN ID = mid + 0x200, 数据为 float (rad/s)"""
    d = list(struct.pack('<f', spd_rad_s)) + [0, 0, 0, 0]
    bus.send(can.Message(arbitration_id=mid + 0x200, data=d[:8], is_extended_id=False))


def main():
    ap = argparse.ArgumentParser(description='DMH3510 定速旋转')
    ap.add_argument('--port',  default='/dev/ttyUSB0', help='SLCAN 串口 (XCAN-USB)')
    ap.add_argument('--speed', type=float, default=5.0, help='目标速度 rad/s')
    ap.add_argument('--id',    type=int,   default=2,   help='电机 CAN ID')
    args = ap.parse_args()

    print(f"[连接] {args.port}  电机ID={args.id}  速度={args.speed} rad/s")
    bus = can.Bus(interface='slcan', channel=args.port, bitrate=1_000_000, timeout=0.5)
    mid = args.id

    try:
        # 1. 失能 + 清错
        special(bus, mid, 0xFD)
        special(bus, mid, 0xFB)

        # 2. 设置 VEL 模式 (寄存器 0x0A = 3)
        write_reg(bus, mid, 0x0A, 3)
        print("[OK] 已切换 VEL 模式")

        # 3. 使能电机
        special(bus, mid, 0xFC)
        time.sleep(0.2)
        print("[OK] 电机使能，旋转中... Ctrl+C 停止\n")

        # 4. 循环发速度指令 + 打印反馈
        while True:
            set_vel(bus, mid, args.speed)
            msg = bus.recv(timeout=0.05)
            if msg and msg.arbitration_id == mid + 0x10 and len(msg.data) >= 5:
                vel_raw = (msg.data[3] << 4) | (msg.data[4] >> 4)
                print(f"\r  反馈 vel_raw={vel_raw:5d}", end='', flush=True)
            time.sleep(0.02)

    except KeyboardInterrupt:
        print("\n[停止]")
    finally:
        set_vel(bus, mid, 0.0)
        time.sleep(0.1)
        special(bus, mid, 0xFD)   # 失能
        bus.shutdown()
        print("[OK] 已失能退出")


if __name__ == '__main__':
    main()
