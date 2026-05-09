#!/usr/bin/env python3
"""
DM 电机 UART 标定脚本 (921600bps, 3-pin GH1.25 串口)
用法: python3 dm_uart_calib.py [/dev/ttyUSBx]
接线: 电机 GH1.25 3-pin → USB-串口适配器 (CH340/CP2102) → USB
"""
import serial, time, sys, struct, threading

PORT = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyUSB0'
BAUD = 921600

print(f"连接 {PORT} @ {BAUD}...")
ser = serial.Serial(PORT, BAUD, timeout=0.2)
time.sleep(0.5)

def monitor_uart(duration=3.0):
    """监听电机 UART 输出"""
    print(f"监听 {duration}s UART 输出:")
    buf = b''
    t = time.time()
    while time.time()-t < duration:
        c = ser.read(256)
        if c:
            buf += c
            try: print(c.decode('utf-8', errors='replace'), end='', flush=True)
            except: print(c.hex(), end=' ')
    print()
    return buf

# 首先监听启动消息
monitor_uart(2.0)

print("\n=== 尝试触发标定 ===")
# DM 调试助手使用的标定触发命令
calib_cmds = [
    b'calib\r\n',
    b'calibrate\r\n',
    b'C\r\n',
    b'c\r\n',
    bytes([0x01]),           # 单字节命令
    bytes([0x63]),           # 'c'
    bytes([0xCA, 0xFB]),    # 可能的二进制命令
]

for cmd in calib_cmds:
    ser.flushInput()
    ser.write(cmd)
    time.sleep(0.3)
    r = ser.read(256)
    if r:
        print(f"  发送 {cmd.hex()} → {r.hex()} = '{r.decode(errors='replace')}'")
        break
    else:
        print(f"  发送 {cmd.hex()} → 无响应")

print("\n若上面看到菜单输出，说明 UART 连接正常。")
print("请告诉我看到什么内容，我来发送正确的标定命令。")
ser.close()
