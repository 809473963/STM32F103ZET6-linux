#!/usr/bin/env python3
"""
XCAN-USB → PTY → slcand → can0 bridge
Usage:
  sudo python3 xcan_bridge.py        # 启动 PTY bridge
  (另一个终端) sudo slcand -o -s8 -S 921600 <PTY_NAME> can0
  sudo ip link set can0 up type can bitrate 1000000
"""
import os, pty, usb.core, usb.util, struct, time, threading, sys, select

VID, PID = 0x2e88, 0x4603
EP_OUT, EP_IN = 0x05, 0x84

# ── 打开 XCAN-USB ─────────────────────────────────────
dev = usb.core.find(idVendor=VID, idProduct=PID)
if dev is None:
    print("[ERROR] 找不到 XCAN-USB"); sys.exit(1)

for i in [0,1]:
    try:
        if dev.is_kernel_driver_active(i): dev.detach_kernel_driver(i)
    except: pass
dev.set_configuration()
dev.ctrl_transfer(0x21, 0x20, 0, 0, struct.pack('<IBBB', 921600, 0, 0, 8))
dev.ctrl_transfer(0x21, 0x22, 0x0003, 0, None)
time.sleep(0.3)
usb.util.claim_interface(dev, 1)
print("[OK] XCAN-USB 已连接")

# ── 创建 PTY ──────────────────────────────────────────
master_fd, slave_fd = pty.openpty()
slave_name = os.ttyname(slave_fd)
print(f"[PTY] 虚拟串口: {slave_name}")
print(f"[PTY] 请在另一个终端运行:")
print(f"      sudo slcand -o -s8 -S 921600 {slave_name} can0")
print(f"      sudo ip link set can0 up type can bitrate 1000000")
print()

# SLCAN 命令处理状态
bus_open = False
buf = bytearray()

def slcan_to_can(cmd: str):
    """解析 SLCAN 命令，返回 (can_id, data_bytes) 或 None"""
    cmd = cmd.strip()
    if not cmd: return None
    c = cmd[0].lower()
    if c == 't' and len(cmd) >= 5:
        try:
            can_id = int(cmd[1:4], 16)
            dlc = int(cmd[4])
            data = bytes(int(cmd[5+i*2:7+i*2], 16) for i in range(dlc))
            return can_id, data
        except: pass
    return None

def send_to_motor(can_id, data):
    """发送 CAN 帧到 XCAN-USB (试多种格式)"""
    # 格式1: STM32 0xAA协议 CMD_DM_RAW_CAN=0x29
    p = list(struct.pack('<H', can_id)) + list(data) + [0]*(8-len(data))
    cs = (10 + 0x29 + sum(p)) & 0xFF
    frame1 = bytes([0xAA, 10, 0x29] + p + [cs])

    # 格式2: 纯 binary [id_lo id_hi dlc data...]
    frame2 = struct.pack('<HB', can_id, len(data)) + bytes(data).ljust(8, b'\x00')

    # 格式3: [0x01 id_hi id_lo dlc data xor]
    hi = (can_id >> 8) & 0x07; lo = can_id & 0xFF
    frame3 = bytes([0x01, hi, lo, len(data)] + list(data) + [0]*(8-len(data)))

    for frame in [frame1, frame2, frame3]:
        try: dev.write(EP_OUT, frame, timeout=100)
        except: pass

def usb_rx_thread():
    """从 XCAN-USB 接收数据，转换为 SLCAN 格式写入 master_fd"""
    rbuf = bytearray()
    while True:
        try:
            raw = bytes(dev.read(EP_IN, 64, timeout=100))
        except: continue
        if not raw: continue
        rbuf.extend(raw)
        # 尝试解析帧: 寻找 t/T 开头的 SLCAN 或 0xAA 头的 binary
        # 直接透传原始数据并打印
        print(f"[USB_RX] {raw.hex()}")
        # 如果是 SLCAN 格式，直接写 PTY
        if any(raw[i:i+1] in (b't', b'T', b'r', b'R') for i in range(min(4,len(raw)))):
            os.write(master_fd, raw)

threading.Thread(target=usb_rx_thread, daemon=True).start()

# ── 主循环：读 PTY (slcand 指令) → 转发到 XCAN-USB ───
print("[Bridge] 等待 slcand 连接... (Ctrl+C 退出)")
while True:
    r, _, _ = select.select([master_fd], [], [], 0.05)
    if r:
        chunk = os.read(master_fd, 256)
        buf.extend(chunk)
        while b'\r' in buf:
            idx = buf.index(b'\r')
            cmd = buf[:idx].decode(errors='ignore')
            buf = buf[idx+1:]
            print(f"[PTY_RX] '{cmd}'")

            c = cmd.strip().lower()
            if c in ('o', 'l'):
                bus_open = True
                os.write(master_fd, b'\r')  # ACK
            elif c == 'c':
                bus_open = False
                os.write(master_fd, b'\r')
            elif c.startswith('s') or c.startswith('z'):
                os.write(master_fd, b'\r')  # ACK bitrate
            elif c.startswith('v') or c.startswith('n'):
                os.write(master_fd, b'V1013\r')  # version
            elif c.startswith('f'):
                os.write(master_fd, b'F00\r')     # status flags
            elif c.startswith('t') and bus_open:
                result = slcan_to_can(cmd)
                if result:
                    cid, data = result
                    send_to_motor(cid, data)
                    os.write(master_fd, b'z\r')  # ACK tx
