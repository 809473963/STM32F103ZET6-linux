#!/usr/bin/env python3
"""DM3510 电机诊断工具 — 排查电机不转问题
用法: python3 dm3510_diag.py [--port /dev/ttyUSB0] [--motor-id 2]

流程:
  1. 扫描 CAN 总线确认电机在线
  2. 读取关键寄存器 (控制模式, FOC参数, 电流环增益等)
  3. MIT 模式测试: 发送不同 kd/v_des 组合, 记录反馈
  4. VEL 模式测试: 切速度模式, 发送速度指令, 记录反馈
  5. 全部数据保存到 CSV + 终端打印摘要
"""

import argparse, csv, os, serial, struct, time, sys
from datetime import datetime

# ── 协议常量 aaaaaaaaaaaaa─────────────────────────────────────
HEADER = 0xAA
CMD_GET_STATUS       = 0x03
CMD_STATUS_REPLY     = 0x83
CMD_SET_ENABLE       = 0x04
CMD_SET_MOTOR_VEL_MRAD = 0x05
CMD_DM_REG_READ      = 0x26
CMD_DM_REG_READ_REPLY = 0xA6
CMD_DM_REG_WRITE     = 0x27
CMD_DM_REG_WRITE_REPLY = 0xA7
CMD_GATEWAY_CONTROL   = 0x20

# ── 寄存器地址 ────────────────────────────────────────────────
REG_RUN_MODE      = 0x01  # 运行状态
REG_CTRL_MODE     = 0x0A  # 控制模式 1=MIT 3=VEL
REG_CUR_KP        = 0x0B  # 电流环 KP
REG_CUR_KI        = 0x0C  # 电流环 KI
REG_SPD_KP        = 0x0D  # 速度环 KP
REG_SPD_KI        = 0x0E  # 速度环 KI
REG_POS_KP        = 0x0F  # 位置环 KP
REG_POS_KI        = 0x10  # 位置环 KI
REG_VMAX           = 0x16  # dq_max
REG_PMAX           = 0x15  # q_max
REG_TMAX           = 0x17  # tau_max
REG_FEEDBACK_ID    = 0x07
REG_RECV_ID        = 0x08

CTRL_MODE_NAMES = {1: "MIT", 2: "POS_VEL", 3: "VEL(Speed)", 4: "Hybrid"}

# ── 帧工具 ────────────────────────────────────────────────────
def make_frame(cmd, payload):
    length = len(payload)
    cs = (length + cmd + sum(payload)) & 0xFF
    return bytes([HEADER, length, cmd]) + bytes(payload) + bytes([cs])

def send(ser, cmd, payload):
    ser.write(make_frame(cmd, payload))

def recv_frame(ser, expected_cmd=None, timeout=0.3):
    """尝试接收一帧, 返回 (cmd, payload) 或 None"""
    deadline = time.time() + timeout
    buf = bytearray()
    while time.time() < deadline:
        chunk = ser.read(max(1, ser.in_waiting))
        if chunk:
            buf.extend(chunk)
        # 尝试解析
        while len(buf) >= 4:
            idx = buf.find(HEADER)
            if idx < 0:
                buf.clear(); break
            if idx > 0:
                del buf[:idx]
            if len(buf) < 3:
                break
            plen = buf[1]
            total = 3 + plen + 1
            if len(buf) < total:
                break
            cmd = buf[2]
            payload = bytes(buf[3:3+plen])
            # checksum
            cs_calc = (plen + cmd + sum(payload)) & 0xFF
            cs_recv = buf[3+plen]
            del buf[:total]
            if cs_calc != cs_recv:
                continue
            if expected_cmd is None or cmd == expected_cmd:
                return (cmd, payload)
    return None

# ── 高级命令 ──────────────────────────────────────────────────
def cmd_enable(ser, motor_id, en):
    send(ser, CMD_SET_ENABLE, [motor_id, 1 if en else 0])
    time.sleep(0.05)

def cmd_set_vel_mrad(ser, motor_id, vel_mrad):
    lo = vel_mrad & 0xFF
    hi = (vel_mrad >> 8) & 0xFF
    if vel_mrad < 0:
        val = vel_mrad & 0xFFFF
        lo = val & 0xFF
        hi = (val >> 8) & 0xFF
    send(ser, CMD_SET_MOTOR_VEL_MRAD, [motor_id, lo, hi])

def cmd_get_status(ser):
    send(ser, CMD_GET_STATUS, [])
    return recv_frame(ser, CMD_STATUS_REPLY, timeout=0.15)

def cmd_read_reg(ser, motor_id, rid):
    send(ser, CMD_DM_REG_READ, [motor_id, rid])
    r = recv_frame(ser, CMD_DM_REG_READ_REPLY, timeout=0.2)
    if r and len(r[1]) >= 6:
        val_bytes = r[1][2:6]
        return struct.unpack('<I', val_bytes)[0]
    return None

def cmd_write_reg(ser, motor_id, rid, value):
    payload = [motor_id, rid] + list(struct.pack('<I', value))
    send(ser, CMD_DM_REG_WRITE, payload)
    time.sleep(0.02)
    recv_frame(ser, CMD_DM_REG_WRITE_REPLY, timeout=0.15)

def cmd_gateway_mit(ser, motor_id, p_des, v_des, kp, kd, t_ff):
    """直接发 gateway 帧控制单个电机 (MIT 模式)"""
    frame = bytearray(39)
    frame[0] = HEADER
    frame[1] = 36  # payload len
    frame[2] = CMD_GATEWAY_CONTROL
    frame[3] = 1  # group A
    frame[4] = 1  # mode MIT
    frame[5] = motor_id
    frame[6] = 1  # enabled
    struct.pack_into('<f', frame, 8, p_des)
    struct.pack_into('<f', frame, 12, v_des)
    struct.pack_into('<f', frame, 16, kp)
    struct.pack_into('<f', frame, 20, kd)
    struct.pack_into('<f', frame, 24, t_ff)
    struct.pack_into('<f', frame, 28, p_des)   # target_position
    struct.pack_into('<f', frame, 32, v_des)   # target_velocity
    frame[36] = 0; frame[37] = 0
    frame[38] = sum(frame[2:38]) & 0xFF
    ser.write(frame)

def parse_status(payload):
    """解析 STATUS_REPLY"""
    if len(payload) < 17:
        return None
    group = payload[0]
    mode = payload[1]
    mid = payload[2]
    enabled = payload[3]
    pos = struct.unpack('<f', payload[4:8])[0]
    vel = struct.unpack('<f', payload[8:12])[0]
    tau = struct.unpack('<f', payload[12:16])[0]
    err = payload[16]
    return {
        'group': group, 'mode': mode, 'id': mid,
        'enabled': enabled, 'pos': pos, 'vel': vel,
        'tau': tau, 'err': err
    }

def u32_to_float(v):
    return struct.unpack('<f', struct.pack('<I', v))[0]

# ── 诊断流程 ──────────────────────────────────────────────────
def read_registers(ser, motor_id):
    """读取关键寄存器并打印"""
    regs = [
        (REG_CTRL_MODE, "控制模式"),
        (REG_CUR_KP,    "电流环 KP"),
        (REG_CUR_KI,    "电流环 KI"),
        (REG_SPD_KP,    "速度环 KP"),
        (REG_SPD_KI,    "速度环 KI"),
        (REG_POS_KP,    "位置环 KP"),
        (REG_POS_KI,    "位置环 KI"),
        (REG_PMAX,      "PMAX (q_max)"),
        (REG_VMAX,      "VMAX (dq_max)"),
        (REG_TMAX,      "TMAX (tau_max)"),
    ]
    results = {}
    print(f"\n{'='*60}")
    print(f"  电机 {motor_id} 寄存器读取")
    print(f"{'='*60}")
    for rid, name in regs:
        val = cmd_read_reg(ser, motor_id, rid)
        if val is not None:
            fval = u32_to_float(val)
            mode_str = ""
            if rid == REG_CTRL_MODE:
                mode_str = f"  → {CTRL_MODE_NAMES.get(val, '未知')}"
            print(f"  [0x{rid:02X}] {name:16s} = {val:10d} (0x{val:08X})  float={fval:.6f}{mode_str}")
            results[rid] = (val, fval)
        else:
            print(f"  [0x{rid:02X}] {name:16s} = 读取失败!")
            results[rid] = None
        time.sleep(0.03)
    return results

def collect_feedback(ser, motor_id, duration_s, label, csv_rows):
    """持续采集反馈数据"""
    t0 = time.time()
    count = 0
    while time.time() - t0 < duration_s:
        r = cmd_get_status(ser)
        if r:
            fb = parse_status(r[1])
            if fb:
                elapsed = time.time() - t0
                row = {
                    'time': f"{elapsed:.3f}",
                    'phase': label,
                    'id': fb['id'],
                    'enabled': fb['enabled'],
                    'pos': f"{fb['pos']:.6f}",
                    'vel': f"{fb['vel']:.6f}",
                    'tau': f"{fb['tau']:.6f}",
                    'err': fb['err'],
                }
                csv_rows.append(row)
                count += 1
                if count % 5 == 0:
                    print(f"    [{label}] t={elapsed:.1f}s  pos={fb['pos']:+.4f}  "
                          f"vel={fb['vel']:+.4f}  tau={fb['tau']:+.4f}  err={fb['err']}")
        time.sleep(0.02)
    return count

def run_diag(ser, motor_id):
    csv_rows = []
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_file = f"dm3510_diag_{motor_id}_{ts}.csv"

    # ── 0. 初始读取 ──
    print("\n" + "━"*60)
    print("  DM3510 电机诊断 - 开始")
    print("━"*60)

    # 先禁用, 清理状态
    cmd_enable(ser, motor_id, False)
    time.sleep(0.3)

    # 读取初始状态
    r = cmd_get_status(ser)
    if r:
        fb = parse_status(r[1])
        if fb:
            print(f"\n  初始状态: id={fb['id']} en={fb['enabled']} "
                  f"pos={fb['pos']:.4f} vel={fb['vel']:.4f} tau={fb['tau']:.4f} err={fb['err']}")

    # ── 1. 读寄存器 ──
    regs = read_registers(ser, motor_id)

    # ── 2. MIT 模式测试 ──
    print(f"\n{'='*60}")
    print("  测试1: MIT 模式 (纯速度: kp=0, kd=N, v_des=M)")
    print(f"{'='*60}")

    # 确保是 MIT 模式
    cmd_write_reg(ser, motor_id, REG_CTRL_MODE, 1)
    time.sleep(0.1)

    # 使能
    cmd_enable(ser, motor_id, True)
    time.sleep(0.3)

    mit_tests = [
        # (v_des, kp, kd, t_ff, duration, label)
        (0.0,   0.0, 1.0, 0.0,  1.0, "MIT_hold_kd1"),
        (5.0,   0.0, 0.5, 0.0,  2.0, "MIT_v5_kd0.5"),
        (5.0,   0.0, 1.0, 0.0,  2.0, "MIT_v5_kd1"),
        (5.0,   0.0, 2.0, 0.0,  2.0, "MIT_v5_kd2"),
        (0.0,   0.0, 0.0, 0.3,  2.0, "MIT_torque_0.3"),
        (0.0,   0.0, 0.0, 0.0,  0.5, "MIT_zero"),
    ]

    for v_des, kp, kd, t_ff, dur, label in mit_tests:
        print(f"\n  >> {label}: v_des={v_des}, kp={kp}, kd={kd}, t_ff={t_ff}")
        t0 = time.time()
        count = 0
        while time.time() - t0 < dur:
            # 持续发 gateway MIT 帧
            cmd_gateway_mit(ser, motor_id, 0.0, v_des, kp, kd, t_ff)
            time.sleep(0.01)
            # 采集反馈
            r = cmd_get_status(ser)
            if r:
                fb = parse_status(r[1])
                if fb:
                    elapsed = time.time() - t0
                    csv_rows.append({
                        'time': f"{elapsed:.3f}", 'phase': label,
                        'id': fb['id'], 'enabled': fb['enabled'],
                        'pos': f"{fb['pos']:.6f}", 'vel': f"{fb['vel']:.6f}",
                        'tau': f"{fb['tau']:.6f}", 'err': fb['err'],
                    })
                    count += 1
                    if count % 10 == 0:
                        print(f"    t={elapsed:.1f}s  pos={fb['pos']:+.4f}  "
                              f"vel={fb['vel']:+.4f}  tau={fb['tau']:+.4f}  err={fb['err']}")

    # 停止
    cmd_gateway_mit(ser, motor_id, 0.0, 0.0, 0.0, 1.0, 0.0)
    time.sleep(0.1)
    cmd_enable(ser, motor_id, False)
    time.sleep(0.3)

    # ── 3. VEL 模式测试 ──
    print(f"\n{'='*60}")
    print("  测试2: VEL 速度模式 (通过 vel_mrad 命令)")
    print(f"{'='*60}")

    # 切到 VEL 模式
    cmd_write_reg(ser, motor_id, REG_CTRL_MODE, 3)
    time.sleep(0.1)

    # 验证模式切换
    v = cmd_read_reg(ser, motor_id, REG_CTRL_MODE)
    print(f"  控制模式寄存器 = {v}  (期望3=VEL)")

    cmd_enable(ser, motor_id, True)
    time.sleep(0.3)

    vel_tests = [
        # (vel_mrad, duration, label)
        (0,     1.0, "VEL_0"),
        (1000,  2.0, "VEL_1rad"),
        (3000,  2.0, "VEL_3rad"),
        (5000,  2.0, "VEL_5rad"),
        (-3000, 2.0, "VEL_-3rad"),
        (0,     1.0, "VEL_stop"),
    ]

    for vel_mrad, dur, label in vel_tests:
        print(f"\n  >> {label}: vel={vel_mrad} mrad/s ({vel_mrad/1000:.1f} rad/s)")
        t0 = time.time()
        count = 0
        while time.time() - t0 < dur:
            cmd_set_vel_mrad(ser, motor_id, vel_mrad)
            time.sleep(0.02)
            r = cmd_get_status(ser)
            if r:
                fb = parse_status(r[1])
                if fb:
                    elapsed = time.time() - t0
                    csv_rows.append({
                        'time': f"{elapsed:.3f}", 'phase': label,
                        'id': fb['id'], 'enabled': fb['enabled'],
                        'pos': f"{fb['pos']:.6f}", 'vel': f"{fb['vel']:.6f}",
                        'tau': f"{fb['tau']:.6f}", 'err': fb['err'],
                    })
                    count += 1
                    if count % 10 == 0:
                        print(f"    t={elapsed:.1f}s  pos={fb['pos']:+.4f}  "
                              f"vel={fb['vel']:+.4f}  tau={fb['tau']:+.4f}  err={fb['err']}")

    # 停止 + 禁用
    cmd_set_vel_mrad(ser, motor_id, 0)
    time.sleep(0.1)
    cmd_enable(ser, motor_id, False)
    time.sleep(0.2)

    # ── 4. 切回 MIT 模式 ──
    cmd_write_reg(ser, motor_id, REG_CTRL_MODE, 1)
    time.sleep(0.05)

    # ── 5. 保存 CSV ──
    if csv_rows:
        fieldnames = ['time', 'phase', 'id', 'enabled', 'pos', 'vel', 'tau', 'err']
        with open(csv_file, 'w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=fieldnames)
            w.writeheader()
            w.writerows(csv_rows)
        print(f"\n{'='*60}")
        print(f"  数据已保存: {csv_file}  ({len(csv_rows)} 条记录)")
        print(f"{'='*60}")
    else:
        print("\n  [警告] 未采集到任何反馈数据!")

    # ── 6. 摘要 ──
    print(f"\n{'='*60}")
    print("  诊断摘要")
    print(f"{'='*60}")

    phases = {}
    for row in csv_rows:
        ph = row['phase']
        if ph not in phases:
            phases[ph] = {'vel_sum': 0, 'tau_sum': 0, 'n': 0, 'vel_max': 0, 'tau_max': 0}
        v = abs(float(row['vel']))
        t = abs(float(row['tau']))
        phases[ph]['vel_sum'] += v
        phases[ph]['tau_sum'] += t
        phases[ph]['vel_max'] = max(phases[ph]['vel_max'], v)
        phases[ph]['tau_max'] = max(phases[ph]['tau_max'], t)
        phases[ph]['n'] += 1

    print(f"  {'测试阶段':20s} {'平均|vel|':>10s} {'最大|vel|':>10s} {'平均|tau|':>10s} {'最大|tau|':>10s}  {'判定'}")
    print(f"  {'-'*20} {'-'*10} {'-'*10} {'-'*10} {'-'*10}  {'-'*6}")
    for ph, d in phases.items():
        avg_v = d['vel_sum'] / d['n'] if d['n'] else 0
        avg_t = d['tau_sum'] / d['n'] if d['n'] else 0
        # 判定: 如果有速度指令但实际速度<0.1 且扭矩<0.01, 认为无响应
        moving = avg_v > 0.1 or avg_t > 0.02
        verdict = "✓ 响应" if moving else "✗ 无响应"
        if "hold" in ph or "zero" in ph or "stop" in ph or "VEL_0" in ph:
            verdict = "— 保持"
        print(f"  {ph:20s} {avg_v:10.4f} {d['vel_max']:10.4f} {avg_t:10.4f} {d['tau_max']:10.4f}  {verdict}")

    print(f"\n  CSV 文件: {os.path.abspath(csv_file)}")
    print(f"  可用 Excel / Python pandas 打开分析速度/扭矩曲线\n")

    return csv_file


def main():
    ap = argparse.ArgumentParser(description="DM3510 电机诊断")
    ap.add_argument('--port', default='/dev/ttyUSB0')
    ap.add_argument('--baud', type=int, default=115200)
    ap.add_argument('--motor-id', type=int, default=2)
    args = ap.parse_args()

    print(f"连接 {args.port} @ {args.baud} ...")
    ser = serial.Serial(args.port, args.baud, timeout=0.1)
    time.sleep(0.5)
    ser.flushInput()

    try:
        csv_file = run_diag(ser, args.motor_id)
    except KeyboardInterrupt:
        print("\n\n  [中断] 紧急停止...")
        cmd_set_vel_mrad(ser, args.motor_id, 0)
        cmd_enable(ser, args.motor_id, False)
        cmd_write_reg(ser, args.motor_id, REG_CTRL_MODE, 1)
    finally:
        ser.close()


if __name__ == '__main__':
    main()
