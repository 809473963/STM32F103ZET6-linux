#!/usr/bin/env python3
"""
XCAN-USB <-> vcan0 双向桥接 (基于 robotcup/damiao.h 协议)
用法: sudo python3 xcan_to_vcan.py
确保 vcan0 已 up: sudo ip link add dev vcan0 type vcan && sudo ip link set vcan0 up
"""
import usb.core, usb.util, struct, time, socket, threading, sys

# ── XCAN-USB ──────────────────────────────────────────
dev = usb.core.find(idVendor=0x2e88, idProduct=0x4603)
if dev is None:
    print("[ERR] 找不到 XCAN-USB"); sys.exit(1)
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

def xcan_build_tx(can_id, data8, cmd=0x03):
    d = bytes(data8[:8]).ljust(8, b'\x00')
    return (struct.pack('<BBBB', 0x55, 0xAA, 0x1E, cmd)
            + struct.pack('<II', 1, 10)
            + struct.pack('<BI', 0, can_id)
            + struct.pack('<BBBB', 0, 8, 0, 0)
            + d + b'\x00')

# ── SocketCAN (vcan0) ──────────────────────────────────
import ctypes
CAN_RAW = 1
sock = socket.socket(socket.PF_CAN, socket.SOCK_RAW, CAN_RAW)
sock.bind(('vcan0',))
sock.setblocking(False)
print("[OK] vcan0 已绑定")

# ── vcan0 → XCAN-USB ──────────────────────────────────
def vcan_to_xcan():
    while True:
        try:
            frame_raw = sock.recv(16)
            if len(frame_raw) < 16: continue
            can_id = struct.unpack_from('<I', frame_raw, 0)[0] & 0x7FFFFFFF
            dlc    = frame_raw[4]
            data   = frame_raw[8:8+dlc]
            dev.write(0x05, xcan_build_tx(can_id, data, cmd=0x03), timeout=100)
        except BlockingIOError: time.sleep(0.001)
        except Exception as e: print(f"[vcan→xcan] {e}")

# ── XCAN-USB → vcan0 ──────────────────────────────────
def xcan_to_vcan():
    buf = bytearray()
    while True:
        try: buf.extend(dev.read(0x84, 256, timeout=200))
        except: pass
        while len(buf) >= 16:
            idx = buf.find(0xAA)
            if idx < 0: buf.clear(); break
            if idx > 0: del buf[:idx]
            if len(buf) < 16: break
            if buf[15] == 0x55:
                cmd    = buf[1]
                can_id = struct.unpack_from('<I', buf, 3)[0]
                data   = bytes(buf[7:15])
                del buf[:16]
                if cmd == 0x11:  # 收到电机回帧
                    frame = struct.pack('<IB3x8s', can_id, 8, b'\x00'*3, data)
                    try: sock.send(frame)
                    except: pass
            else: del buf[:1]

t1 = threading.Thread(target=vcan_to_xcan, daemon=True)
t2 = threading.Thread(target=xcan_to_vcan, daemon=True)
t1.start(); t2.start()
print("[Bridge] XCAN-USB ↔ vcan0 桥接运行中... Ctrl+C 退出")
try:
    while True: time.sleep(1)
except KeyboardInterrupt:
    print("退出")
    usb.util.release_interface(dev, 1)
