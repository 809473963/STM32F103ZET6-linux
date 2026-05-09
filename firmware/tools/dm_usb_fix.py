"""
DM3510 直接 USB 修复脚本 (pyusb + SLCAN, 绕过驱动)
"""
import usb.core, usb.util
import struct, time, sys

VID, PID = 0x2e88, 0x4603
MOTOR_ID = 2
EP_OUT   = 0x05
EP_IN    = 0x84

# ── 连接设备 ─────────────────────────────────────────
dev = usb.core.find(idVendor=VID, idProduct=PID)
if dev is None:
    print('[ERROR] 未找到 XCAN-USB 设备')
    sys.exit(1)

# 如果内核已占用接口则先释放
for iface in [0, 1]:
    try:
        if dev.is_kernel_driver_active(iface):
            dev.detach_kernel_driver(iface)
            print(f'  已释放内核驱动 iface={iface}')
    except Exception:
        pass

try:
    dev.set_configuration()
except Exception as e:
    print(f'  set_configuration: {e}')

try:
    usb.util.claim_interface(dev, 1)
    print('[OK] 已声明 CDC Data 接口')
except Exception as e:
    print(f'[WARN] claim_interface: {e}')

def write_raw(data: bytes):
    try:
        dev.write(EP_OUT, data, timeout=200)
    except Exception as e:
        print(f'  write error: {e}')

def read_raw(timeout_ms=300) -> bytes:
    try:
        return bytes(dev.read(EP_IN, 64, timeout=timeout_ms))
    except usb.core.USBTimeoutError:
        return b''
    except Exception as e:
        print(f'  read error: {e}')
        return b''

# ── SLCAN 初始化 ──────────────────────────────────────
print('初始化 SLCAN (S8=1Mbps) ...')
write_raw(b'C\r')      # 关闭（如已开）
time.sleep(0.1)
read_raw(100)
write_raw(b'S8\r')     # 1 Mbps
time.sleep(0.1)
r = read_raw(200)
print(f'  S8 回复: {r}')
write_raw(b'O\r')      # 打开总线
time.sleep(0.1)
r = read_raw(200)
print(f'  O  回复: {r}')

def slcan_send(std_id: int, data8: list):
    """发送标准 CAN 帧 (SLCAN 格式: tIIILDD..DD\r)"""
    s = 't%03X%d' % (std_id, len(data8))
    s += ''.join('%02X' % b for b in data8) + '\r'
    write_raw(s.encode())

def slcan_recv(timeout_ms=400) -> list:
    """返回收到的 CAN 帧列表 [(id, data), ...]"""
    raw = read_raw(timeout_ms)
    frames = []
    if not raw:
        return frames
    for token in raw.decode(errors='ignore').split('\r'):
        token = token.strip()
        if token.startswith('t') and len(token) >= 5:
            try:
                fid  = int(token[1:4], 16)
                dlc  = int(token[4], 16)
                data = bytes(int(token[5+i*2:7+i*2], 16) for i in range(dlc))
                frames.append((fid, data))
            except Exception:
                pass
    return frames

def special(cmd_last):
    slcan_send(MOTOR_ID, [0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,cmd_last])
    time.sleep(0.05)

def reg_write_u32(rid, val_u32):
    data = [MOTOR_ID&0xFF, (MOTOR_ID>>8)&0xFF, 0x55, rid,
            val_u32&0xFF, (val_u32>>8)&0xFF,
            (val_u32>>16)&0xFF, (val_u32>>24)&0xFF]
    slcan_send(0x7FF, data)
    time.sleep(0.08)

def reg_read(rid):
    data = [MOTOR_ID&0xFF, 0x00, 0x33, rid, 0,0,0,0]
    slcan_send(0x7FF, data)
    for _ in range(10):
        for fid, fdata in slcan_recv(100):
            if fid == (MOTOR_ID + 0x10) and len(fdata) >= 8 and fdata[2] == 0x33:
                uv = struct.unpack('<I', fdata[4:8])[0]
                fv = struct.unpack('<f', fdata[4:8])[0]
                return fv, uv
    return None, None

def f2u(f): return struct.unpack('<I', struct.pack('<f', f))[0]

# ── 1. 读当前状态 ─────────────────────────────────────
print('\n=== 当前参数 ===')
for rid, name in [(0x0A,'ctrl_mode'),(0x0B,'Cur_KP'),(0x0C,'Cur_KI'),
                  (0x19,'Vel_KP'),(0x1A,'Vel_KI'),(0x18,'cur_bw')]:
    fv, uv = reg_read(rid)
    if uv is not None:
        print('  0x%02X %-10s = %-14g  0x%08X' % (rid, name, fv, uv))
    else:
        print('  0x%02X %-10s = 超时' % (rid, name))
    time.sleep(0.05)

# ── 2. 失能 + 清错 ────────────────────────────────────
print('\n>>> 失能 + 清错')
special(0xFD); time.sleep(0.2)
special(0xFB); time.sleep(0.2)

# ── 3. 试所有候选标定触发命令 ─────────────────────────
print('\n=== 候选标定触发 0xFA→0xF0 ===')
for last in range(0xFA, 0xEF, -1):
    slcan_recv(50)          # 清空缓冲
    print('  0x%02X ...' % last, end='', flush=True)
    special(last)
    moved = False
    t0 = time.time()
    while time.time() - t0 < 3.0:
        for fid, fdata in slcan_recv(150):
            if fid == MOTOR_ID + 0x10 and len(fdata) >= 6:
                # MIT 反馈: pos(2)+vel(2)+cur(2)
                vel_raw = (fdata[3] << 4) | (fdata[4] >> 4)
                if vel_raw > 50:
                    print(' 电机有动作! vel_raw=%d' % vel_raw)
                    moved = True
        if moved: break
    if not moved:
        print(' 无响应')
    special(0xFD); time.sleep(0.3)

# ── 4. 扫描寄存器找 Rs/Ls ─────────────────────────────
print('\n=== 扫描 Rs/Ls/Psi 候选地址 ===')
TARGET = {0x4B16B806: 'Rs=9873510mΩ', 0x4A20E638: 'Ls=1316230uH'}
for rid in list(range(0x1C, 0x36)) + list(range(0x60, 0x80)):
    fv, uv = reg_read(rid)
    if uv is not None and uv != 0:
        note = TARGET.get(uv, '')
        print('  0x%02X  0x%08X  %-12g  %s' % (rid, uv, fv, note))
    time.sleep(0.03)

# ── 5. 强制写电流环 (无保存) ──────────────────────────
print('\n=== 写 Cur_KP/KI 到 RAM ===')
special(0xFD); time.sleep(0.2)
for rid, name, val in [(0x0B,'Cur_KP',0.5),(0x0C,'Cur_KI',0.01)]:
    reg_write_u32(rid, f2u(val))
    time.sleep(0.1)
    fv, uv = reg_read(rid)
    print('  写 0x%02X %s=%.3f → 回读 %s' % (rid, name, val, fv))

# ── 6. 速度测试 ───────────────────────────────────────
print('\n=== 速度测试 5s ===')
special(0xFC); time.sleep(0.2)  # enable
vel_data = list(struct.pack('<f', 5.0)) + [0,0,0,0]
t0 = time.time()
while time.time() - t0 < 5.0:
    slcan_send(MOTOR_ID + 0x200, vel_data[:8])
    for fid, fdata in slcan_recv(100):
        if fid == MOTOR_ID + 0x10 and len(fdata) >= 8:
            pos_raw = (fdata[1]<<8)|fdata[2]
            vel_raw = (fdata[3]<<4)|(fdata[4]>>4)
            print('  fb: pos_raw=%5d  vel_raw=%4d' % (pos_raw, vel_raw))

special(0xFD)  # disable

# ── 关闭总线 ─────────────────────────────────────────
write_raw(b'C\r'); time.sleep(0.1)
usb.util.release_interface(dev, 1)
print('\n完成')
