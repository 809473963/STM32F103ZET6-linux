"""
DM3510 Motor Parameter Fix Script (Windows + XCAN-USB)
Run: python dm_windows_fix.py
Requires: pip install python-can
"""
import time
import struct
import sys

try:
    import can
except ImportError:
    print("请先安装: pip install python-can")
    sys.exit(1)

# ── 配置 ──────────────────────────────────────────────
CHANNEL  = 'COM3'       # XCAN-USB 串口号
BITRATE  = 1000000      # 1 Mbps
MOTOR_ID = 2            # 电机 CAN ID
# ──────────────────────────────────────────────────────

def open_bus():
    # XCAN-USB 走 SLCAN 协议
    try:
        bus = can.Bus(interface='slcan', channel=CHANNEL, bitrate=BITRATE, timeout=0.5)
        print(f'[OK] 已连接 {CHANNEL} (slcan)')
        return bus
    except Exception as e:
        print(f'[FAIL] slcan 失败: {e}')
        print('尝试 socketcan/pcan ...')
    try:
        bus = can.Bus(interface='pcan', channel='PCAN_USBBUS1', bitrate=BITRATE)
        print('[OK] 已连接 PCAN')
        return bus
    except Exception as e:
        print(f'[FAIL] {e}')
        sys.exit(1)

def send(bus, std_id, data8):
    msg = can.Message(arbitration_id=std_id, data=data8, is_extended_id=False)
    try:
        bus.send(msg)
    except Exception as e:
        print(f'  发送失败: {e}')

def recv(bus, timeout=0.4):
    deadline = time.time() + timeout
    while time.time() < deadline:
        msg = bus.recv(timeout=deadline - time.time())
        if msg:
            return msg
    return None

def special(bus, cmd_last):
    send(bus, MOTOR_ID, [0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,cmd_last])

def reg_write(bus, rid, value_u32):
    data = [
        MOTOR_ID & 0xFF, (MOTOR_ID >> 8) & 0xFF,
        0x55, rid,
        (value_u32 >>  0) & 0xFF,
        (value_u32 >>  8) & 0xFF,
        (value_u32 >> 16) & 0xFF,
        (value_u32 >> 24) & 0xFF,
    ]
    send(bus, 0x7FF, data)
    time.sleep(0.05)

def reg_read(bus, rid):
    data = [MOTOR_ID & 0xFF, (MOTOR_ID >> 8) & 0xFF, 0x33, rid, 0,0,0,0]
    send(bus, 0x7FF, data)
    deadline = time.time() + 0.4
    while time.time() < deadline:
        msg = bus.recv(timeout=0.1)
        if msg and msg.arbitration_id == (MOTOR_ID + 0x10):
            if len(msg.data) >= 8 and msg.data[2] == 0x33:
                uv = struct.unpack('<I', bytes(msg.data[4:8]))[0]
                fv = struct.unpack('<f', bytes(msg.data[4:8]))[0]
                return fv, uv
    return None, None

def float_u32(f):
    return struct.unpack('<I', struct.pack('<f', f))[0]

# ═══════════════════════════════════════════════════════
bus = open_bus()
time.sleep(0.3)

# ── 1. 读当前状态 ──────────────────────────────────────
print('\n=== 当前参数 ===')
for rid, name in [(0x0A,'ctrl_mode'),(0x0B,'Cur_KP'),(0x0C,'Cur_KI'),
                  (0x19,'Vel_KP'),(0x1A,'Vel_KI'),(0x18,'cur_bw')]:
    fv, uv = reg_read(bus, rid)
    print(f'  0x{rid:02X} {name:12s} = {fv}  (0x{uv:08X})' if uv is not None else f'  0x{rid:02X} {name:12s} = 读取超时')
    time.sleep(0.05)

# ── 2. 失能 + 清错 ────────────────────────────────────
print('\n>>> 失能 + 清错...')
special(bus, 0xFD)   # disable
time.sleep(0.2)
special(bus, 0xFB)   # clear error
time.sleep(0.2)

# ── 3. 尝试各候选标定触发字节 ────────────────────────
print('\n=== 标定触发尝试 (0xF0-0xFA) ===')
for last in range(0xFA, 0xEF, -1):
    print(f'  >> 发送 0x{last:02X} ...', end='', flush=True)
    special(bus, last)
    # 监听 3 秒
    t0 = time.time()
    responded = False
    while time.time() - t0 < 3.0:
        msg = recv(bus, 0.2)
        if msg and msg.arbitration_id == (MOTOR_ID + 0x10):
            # 有反馈帧
            if len(msg.data) >= 8:
                vel_raw = (msg.data[4] << 8) | msg.data[5]
                if vel_raw > 100:   # 速度非零 → 电机在动
                    print(f' 电机有动作! vel_raw={vel_raw}')
                    responded = True
                    break
    if not responded:
        print(' 无响应')
    special(bus, 0xFD)  # 每次后 disable
    time.sleep(0.3)

# ── 4. 直接向 0x7FF 写 Rs/Ls 候选寄存器 ─────────────
print('\n=== 直接写 Rs/Ls (寻找正确地址) ===')
# DM3510 参考值: Rs=300mΩ, Ls=100uH, Ψf=0.005Wb
RS_VAL  = float_u32(300.0)   # 300 mΩ
LS_VAL  = float_u32(100.0)   # 100 μH
PSI_VAL = float_u32(0.005)   # 0.005 Wb

candidate_rids = [0x18, 0x19, 0x1C, 0x1D, 0x1E, 0x30, 0x31, 0x32, 0x33, 0x34,
                  0x40, 0x41, 0x42, 0x50, 0x51, 0x52, 0x60, 0x61, 0x62]

print('  先读这些地址看现有值:')
for rid in candidate_rids:
    fv, uv = reg_read(bus, rid)
    if uv is not None and uv != 0:
        print(f'    0x{rid:02X} = float {fv:.4f}  uint 0x{uv:08X}')
    time.sleep(0.03)

# ── 5. 强制写电流环 (不保存, 直接 RAM) ──────────────
print('\n=== 强制写电流环参数到 RAM ===')
special(bus, 0xFD)  # disable 确保安全
time.sleep(0.2)

for rid, name, val in [
    (0x0B, 'Cur_KP', 0.5),
    (0x0C, 'Cur_KI', 0.01),
]:
    reg_write(bus, rid, float_u32(val))
    time.sleep(0.1)
    fv, uv = reg_read(bus, rid)
    print(f'  写 0x{rid:02X} {name}={val} → 回读={fv}  (期望={val})')

# ── 6. 测试：使能 + 速度命令 ─────────────────────────
print('\n=== 速度测试 (5 秒) ===')
special(bus, 0xFC)  # enable
time.sleep(0.2)

# VEL 模式: 发到 can_id + 0x200, 数据为 float 速度 (rad/s)
vel_id = MOTOR_ID + 0x200
vel_bytes = list(struct.pack('<f', 5.0))   # 5 rad/s
vel_bytes += [0] * (8 - len(vel_bytes))

t0 = time.time()
while time.time() - t0 < 5.0:
    send(bus, vel_id, vel_bytes)
    msg = recv(bus, 0.1)
    if msg and msg.arbitration_id == (MOTOR_ID + 0x10):
        d = msg.data
        if len(d) >= 8:
            pos_raw = (d[1] << 8) | d[2]
            vel_raw = (d[3] << 4) | (d[4] >> 4)
            print(f'  反馈: pos_raw={pos_raw}  vel_raw={vel_raw}')

special(bus, 0xFD)  # disable
bus.shutdown()
print('\n完成')
