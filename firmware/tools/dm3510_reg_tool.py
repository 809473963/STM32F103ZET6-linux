#!/usr/bin/env python3
"""
DM3510 电机寄存器读写工具
通过 STM32 CAN 桥接，读写 DM 电机内部寄存器，用于修复参数标定后的错误参数。

用法:
  python3 dm3510_reg_tool.py --port /dev/ttyUSB0 scan   2     # 扫描电机2所有已知寄存器
  python3 dm3510_reg_tool.py --port /dev/ttyUSB0 read   2 0x20 # 读电机2的寄存器0x20
  python3 dm3510_reg_tool.py --port /dev/ttyUSB0 write  2 0x20 <float_value>
  python3 dm3510_reg_tool.py --port /dev/ttyUSB0 save   2     # 保存到Flash
  python3 dm3510_reg_tool.py --port /dev/ttyUSB0 repair 2     # 一键修复DM3510参数
"""
import sys, struct, time, argparse, serial

HEADER = 0xAA
CMD_DM_REG_READ   = 0x26
CMD_DM_REG_WRITE  = 0x27
CMD_DM_REG_SAVE   = 0x28
CMD_DM_RAW_CAN    = 0x29
CMD_DM_REG_READ_REPLY  = 0xA6
CMD_DM_REG_WRITE_REPLY = 0xA7

# ── DM 电机常见寄存器地址 (基于达妙公开协议) ──
REG_MAP = {
    0x00: "UV_Value (欠压保护值)",
    0x01: "SAVE/RESET (写1保存, 写2恢复出厂)",
    0x02: "KT_Value (扭矩系数)",
    0x03: "OT_Value (过温保护)",
    0x04: "OC_Value (过流保护)",
    0x05: "ACC (加速度)",
    0x06: "DEC (减速度)",
    0x07: "MAX_SPD (最大速度)",
    0x08: "MST_ID (反馈ID/Master)",
    0x09: "ESC_ID (CAN接收ID)",
    0x0A: "CTRL_MODE (控制模式)",
    0x0B: "LIMIT_SPD",
    0x0C: "CUR_KP (电流环KP)",
    0x0D: "CUR_KI (电流环KI)",
    0x0E: "SPD_KP (速度环KP)",
    0x0F: "SPD_KI (速度环KI)",
    0x10: "POS_KP (位置环KP)",
    0x11: "POS_KI (位置环KI)",
    0x12: "POS_KD",
    0x13: "POS_FF",
    0x14: "CAN_BAUD",
    0x15: "SUB_VER",
    0x16: "CTRL_WORD",
    # 电机电气参数 (关键!)
    0x30: "Rs (相电阻 mΩ)",
    0x31: "Ls (相电感 μH)",
    0x32: "Flux (磁链 Ψf)",
    # 替代地址 (某些固件版本)
    0x20: "Rs_alt",
    0x21: "Ls_alt",
    0x22: "Flux_alt",
}

# DM3510 规格书正确参数
DM3510_CORRECT = {
    "Rs_mOhm": 2250.0,      # 相电阻 2.25Ω = 2250mΩ
    "Ls_uH":   1670.0,      # 相电感 1.67μH (or mH→1670μH, 需确认)
    "Psi_f":   0.031,       # 磁链 ~0.031 Wb (Kt=0.164, p=3.5, Ψf=Kt/(1.5*p))
}


def make_frame(cmd, payload):
    length = len(payload)
    cs = (length + cmd + sum(payload)) & 0xFF
    return bytes([HEADER, length, cmd, *payload, cs])


def wait_reply(ser, expected_cmd, timeout=0.5):
    """等待 STM32 回复帧"""
    buf = bytearray()
    t0 = time.time()
    while time.time() - t0 < timeout:
        d = ser.read(64)
        if d:
            buf.extend(d)
        # 尝试解析
        while len(buf) >= 4:
            idx = buf.find(HEADER)
            if idx < 0:
                buf.clear(); break
            if idx > 0:
                del buf[:idx]
            if len(buf) < 4:
                break
            length = buf[1]
            total = 3 + length + 1  # header + length + cmd + payload + cs
            if len(buf) < total:
                break
            cmd = buf[2]
            payload = bytes(buf[3:3+length])
            cs_got = buf[3+length]
            cs_calc = (length + cmd + sum(payload)) & 0xFF
            del buf[:total]
            if cs_got == cs_calc and cmd == expected_cmd:
                return payload
    return None


def reg_read(ser, motor_id, rid):
    """读取一个寄存器，返回 (success, value_u32) or (False, 0)"""
    ser.write(make_frame(CMD_DM_REG_READ, [motor_id & 0xFF, rid & 0xFF]))
    rpl = wait_reply(ser, CMD_DM_REG_READ_REPLY, timeout=0.3)
    if rpl and len(rpl) >= 7 and rpl[2] == 1:
        val = struct.unpack("<I", rpl[3:7])[0]
        return True, val
    return False, 0


def reg_read_float(ser, motor_id, rid):
    """读取寄存器并解释为 float"""
    ok, raw = reg_read(ser, motor_id, rid)
    if ok:
        f = struct.unpack("<f", struct.pack("<I", raw))[0]
        return True, f
    return False, 0.0


def reg_write_float(ser, motor_id, rid, fval):
    """写 float 到寄存器"""
    raw = struct.unpack("<I", struct.pack("<f", fval))[0]
    payload = [motor_id & 0xFF, rid & 0xFF] + list(struct.pack("<I", raw))
    ser.write(make_frame(CMD_DM_REG_WRITE, payload))
    rpl = wait_reply(ser, CMD_DM_REG_WRITE_REPLY, timeout=0.3)
    if rpl and len(rpl) >= 7 and rpl[2] == 1:
        return True
    return False


def reg_write_u32(ser, motor_id, rid, val):
    """写 uint32 到寄存器"""
    payload = [motor_id & 0xFF, rid & 0xFF] + list(struct.pack("<I", val))
    ser.write(make_frame(CMD_DM_REG_WRITE, payload))
    rpl = wait_reply(ser, CMD_DM_REG_WRITE_REPLY, timeout=0.3)
    if rpl and len(rpl) >= 7 and rpl[2] == 1:
        return True
    return False


def reg_save(ser, motor_id):
    """保存参数到 Flash"""
    ser.write(make_frame(CMD_DM_REG_SAVE, [motor_id & 0xFF]))
    time.sleep(0.2)
    print(f"  [SAVE] 已发送保存命令到电机 {motor_id}")


def cmd_scan(ser, motor_id):
    """扫描所有已知寄存器"""
    print(f"\n═══ 扫描电机 {motor_id} 寄存器 ═══\n")
    # 先扫描常用 + 电气参数
    rids = sorted(set(list(range(0x00, 0x17)) + list(range(0x20, 0x35))))
    for rid in rids:
        ok_u, raw = reg_read(ser, motor_id, rid)
        if ok_u:
            f = struct.unpack("<f", struct.pack("<I", raw))[0]
            name = REG_MAP.get(rid, "?")
            print(f"  [0x{rid:02X}] {name:30s}  raw=0x{raw:08X}  uint={raw:>12d}  float={f:.6f}")
        time.sleep(0.05)
    print()


def cmd_read(ser, motor_id, rid):
    ok, raw = reg_read(ser, motor_id, rid)
    if ok:
        f = struct.unpack("<f", struct.pack("<I", raw))[0]
        name = REG_MAP.get(rid, "?")
        print(f"  [0x{rid:02X}] {name}  raw=0x{raw:08X}  uint={raw}  float={f:.6f}")
    else:
        print(f"  [0x{rid:02X}] 读取失败")


def cmd_write(ser, motor_id, rid, fval):
    print(f"  写入 电机{motor_id} 寄存器0x{rid:02X} = {fval}")
    ok = reg_write_float(ser, motor_id, rid, fval)
    if ok:
        print(f"  ✓ 写入成功")
        # 回读验证
        time.sleep(0.05)
        ok2, f2 = reg_read_float(ser, motor_id, rid)
        if ok2:
            print(f"  回读验证: {f2:.6f}")
    else:
        print(f"  ✗ 写入失败")


def cmd_repair(ser, motor_id):
    """尝试修复 DM3510 的电气参数"""
    print(f"\n═══ 修复电机 {motor_id} (DM3510) 参数 ═══\n")
    print("  规格书参数:")
    print(f"    Rs = {DM3510_CORRECT['Rs_mOhm']:.1f} mΩ  ({DM3510_CORRECT['Rs_mOhm']/1000:.3f} Ω)")
    print(f"    Ls = {DM3510_CORRECT['Ls_uH']:.1f} μH")
    print(f"    Ψf = {DM3510_CORRECT['Psi_f']:.4f} Wb")
    print()

    # 先扫描找到实际的寄存器地址
    print("  [1/4] 扫描当前参数 (尝试 0x20-0x22 和 0x30-0x32)...")
    rs_rid = ls_rid = psi_rid = None

    for rid in [0x30, 0x20]:
        ok, f = reg_read_float(ser, motor_id, rid)
        if ok and abs(f) > 100:  # Rs 应该 > 100 mΩ
            rs_rid = rid
            print(f"    Rs 在 0x{rid:02X}: 当前值 = {f:.1f} mΩ")
            break
        time.sleep(0.05)

    for rid in [0x31, 0x21]:
        ok, f = reg_read_float(ser, motor_id, rid)
        if ok and abs(f) > 0.1:
            ls_rid = rid
            print(f"    Ls 在 0x{rid:02X}: 当前值 = {f:.1f} μH")
            break
        time.sleep(0.05)

    for rid in [0x32, 0x22]:
        ok, f = reg_read_float(ser, motor_id, rid)
        if ok:
            psi_rid = rid
            print(f"    Ψf 在 0x{rid:02X}: 当前值 = {f:.6f} Wb")
            break
        time.sleep(0.05)

    if not (rs_rid and ls_rid and psi_rid):
        print("\n  ✗ 未能定位所有参数寄存器!")
        print("    先运行 scan 命令查看完整寄存器表，手动确认地址后再重试。")
        return

    print(f"\n  [2/4] 写入正确参数...")
    ok1 = reg_write_float(ser, motor_id, rs_rid, DM3510_CORRECT["Rs_mOhm"])
    time.sleep(0.05)
    ok2 = reg_write_float(ser, motor_id, ls_rid, DM3510_CORRECT["Ls_uH"])
    time.sleep(0.05)
    ok3 = reg_write_float(ser, motor_id, psi_rid, DM3510_CORRECT["Psi_f"])
    time.sleep(0.05)
    print(f"    Rs: {'✓' if ok1 else '✗'}  Ls: {'✓' if ok2 else '✗'}  Ψf: {'✓' if ok3 else '✗'}")

    if not (ok1 and ok2 and ok3):
        print("\n  ✗ 部分写入失败！不保存。")
        return

    print(f"\n  [3/4] 回读验证...")
    time.sleep(0.1)
    _, rs_v = reg_read_float(ser, motor_id, rs_rid)
    time.sleep(0.05)
    _, ls_v = reg_read_float(ser, motor_id, ls_rid)
    time.sleep(0.05)
    _, psi_v = reg_read_float(ser, motor_id, psi_rid)
    print(f"    Rs = {rs_v:.1f} mΩ  (期望 {DM3510_CORRECT['Rs_mOhm']:.1f})")
    print(f"    Ls = {ls_v:.1f} μH  (期望 {DM3510_CORRECT['Ls_uH']:.1f})")
    print(f"    Ψf = {psi_v:.6f} Wb  (期望 {DM3510_CORRECT['Psi_f']:.6f})")

    # 检查是否写入成功
    rs_ok = abs(rs_v - DM3510_CORRECT["Rs_mOhm"]) < 1.0
    ls_ok = abs(ls_v - DM3510_CORRECT["Ls_uH"]) < 1.0
    psi_ok = abs(psi_v - DM3510_CORRECT["Psi_f"]) < 0.001

    if rs_ok and ls_ok and psi_ok:
        print(f"\n  [4/4] 保存到 Flash...")
        reg_save(ser, motor_id)
        print("\n  ✓ 修复完成！断电重启电机后生效。")
    else:
        print("\n  ✗ 回读不匹配，可能寄存器地址不对。不保存。")
        print("    请用 scan 命令查看完整寄存器表。")


def send_raw_can(ser, std_id, data8):
    """发送原始 CAN 帧"""
    payload = list(struct.pack("<H", std_id)) + list(data8[:8])
    while len(payload) < 10:
        payload.append(0)
    ser.write(make_frame(CMD_DM_RAW_CAN, payload))
    time.sleep(0.05)


def cmd_canopen_restore(ser, node_id):
    """通过 CANOpen SDO 写 index 1011 sub 1 = 'load' 恢复出厂"""
    cob_id = 0x600 + node_id
    # SDO expedited download, 4 bytes: cmd=0x23, index=0x1011(LE), sub=0x01, data='load'(LE)
    sdo_data = [0x23, 0x11, 0x10, 0x01, 0x6C, 0x6F, 0x61, 0x64]
    print(f"  发送 CANOpen SDO: COB-ID=0x{cob_id:03X}, Index=0x1011, Sub=0x01, Data='load'")
    send_raw_can(ser, cob_id, sdo_data)
    print(f"  ✓ 已发送。断电重启电机后生效。")


def main():
    ap = argparse.ArgumentParser(description="DM3510 电机寄存器读写工具")
    ap.add_argument('--port', default='/dev/ttyUSB0', help='STM32 串口')
    ap.add_argument('--baud', type=int, default=115200)
    sub = ap.add_subparsers(dest='cmd')

    p_scan = sub.add_parser('scan', help='扫描所有寄存器')
    p_scan.add_argument('motor_id', type=int)

    p_read = sub.add_parser('read', help='读寄存器')
    p_read.add_argument('motor_id', type=int)
    p_read.add_argument('rid', type=lambda x: int(x, 0))

    p_write = sub.add_parser('write', help='写寄存器(float)')
    p_write.add_argument('motor_id', type=int)
    p_write.add_argument('rid', type=lambda x: int(x, 0))
    p_write.add_argument('value', type=float)

    p_save = sub.add_parser('save', help='保存到Flash')
    p_save.add_argument('motor_id', type=int)

    p_repair = sub.add_parser('repair', help='一键修复DM3510参数')
    p_repair.add_argument('motor_id', type=int)

    p_factory = sub.add_parser('factory', help='尝试恢复出厂设置(写寄存器0x01=2)')
    p_factory.add_argument('motor_id', type=int)

    p_canopen = sub.add_parser('canopen_restore', help='CANOpen SDO恢复出厂(index 1011)')
    p_canopen.add_argument('node_id', type=int, help='CANOpen Node ID (通常=电机CAN ID)')

    args = ap.parse_args()
    if not args.cmd:
        ap.print_help(); return

    ser = serial.Serial(args.port, args.baud, timeout=0.1)
    ser.setDTR(False); ser.setRTS(False)
    time.sleep(0.5)
    ser.flushInput()

    if args.cmd == 'scan':
        cmd_scan(ser, args.motor_id)
    elif args.cmd == 'read':
        cmd_read(ser, args.motor_id, args.rid)
    elif args.cmd == 'write':
        cmd_write(ser, args.motor_id, args.rid, args.value)
    elif args.cmd == 'save':
        reg_save(ser, args.motor_id)
    elif args.cmd == 'repair':
        cmd_repair(ser, args.motor_id)
    elif args.cmd == 'factory':
        print(f"  尝试发送恢复出厂设置命令 (0x01=2) 到电机 {args.motor_id}...")
        ok = reg_write_u32(ser, args.motor_id, 0x01, 2)
        if ok:
            print("  ✓ 命令已发送。断电重启电机。")
        else:
            print("  ✗ 命令发送失败或无回复。")
    elif args.cmd == 'canopen_restore':
        cmd_canopen_restore(ser, args.node_id)

    ser.close()


if __name__ == '__main__':
    main()
