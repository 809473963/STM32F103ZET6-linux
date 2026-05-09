#!/usr/bin/env python3
"""PC keyboard teleop for STM32 motor controller over serial.

Protocol frame:
  [0xAA] [LEN] [CMD] [DATA...] [CHECKSUM]
  CHECKSUM = (LEN + CMD + DATA bytes) & 0xFF
"""

import argparse
import atexit
import csv
import glob
import math
import os
import select
import signal
import struct
import subprocess
import sys
import termios
import tty
import time
from datetime import datetime

import serial

HEADER = 0xAA
CMD_SET_MOTOR_SPEED = 0x01
CMD_STOP_ALL = 0x02
CMD_SET_ENABLE = 0x04
CMD_SET_MOTOR_VEL_MRAD = 0x05
CMD_GET_STATUS = 0x03
CMD_DM_SCAN_IDS = 0x21
CMD_DM_SET_ID = 0x22
CMD_SET_GRAVITY_COMP = 0x24
CMD_REZERO_GRAVITY = 0x25
CMD_GRIPPER_JOG = 0x30
CMD_GRIPPER_SET_ZERO = 0x31
CMD_GRIPPER_SAVE_CFG = 0x32
CMD_GRIPPER_TRUST_ZERO = 0x33
CMD_GRIPPER_SET_OPEN = 0x34
CMD_GRIPPER_GET_STATUS = 0x35

CMD_STATUS_REPLY = 0x83
CMD_DM_SCAN_IDS_REPLY = 0xA1
CMD_DM_SET_ID_REPLY = 0xA2
CMD_GRIPPER_STATUS_REPLY = 0xB0

DIR_FORWARD = 1
DIR_BACKWARD = 0
DIR_STOP = 2

# Must match firmware side mapping in protocol.c.
VEL_MAX_RAD_S = 14.0

# Motor-ID to URDF joint mapping (must match firmware wiring and robot assembly).
# Specs (三款电机参数对比表.xlsx):
#   M24    (ID5)    : peak 23.5 N.m, 6:1 reduction,    rated 140 rpm, base yaw Z
#   DM4310 (ID1)   : peak  7.0 N.m, 10:1 reduction,   rated 120 rpm, base pitch
#   DH3510 (ID2-4) : peak  0.45 N.m, 1:1 DIRECT DRIVE, rated 500 rpm, arm joints
MOTOR_JOINT_MAP = {
    1: {"joint": "joint2", "type": "DM4310", "role": "base_pitch"},
    2: {"joint": "joint3", "type": "DH3510", "role": "mid_joint_1"},
    3: {"joint": "joint4", "type": "DH3510", "role": "mid_joint_2"},
    4: {"joint": "joint6", "type": "DH3510", "role": "wrist_or_gripper_carrier"},
    5: {"joint": "joint1", "type": "M24",    "role": "base_yaw_z"},
}


class RawTerminal:
    def __init__(self):
        self._fd = sys.stdin.fileno()
        self._old = termios.tcgetattr(self._fd)

    def __enter__(self):
        tty.setraw(self._fd)
        # setraw clears ISIG, which disables Ctrl+C → SIGINT.
        # Re-enable it so the registered signal handler fires immediately.
        mode = termios.tcgetattr(self._fd)
        mode[3] |= termios.ISIG
        termios.tcsetattr(self._fd, termios.TCSADRAIN, mode)
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        termios.tcsetattr(self._fd, termios.TCSADRAIN, self._old)


def checksum(length, cmd, payload):
    return (length + cmd + sum(payload)) & 0xFF


def make_frame(cmd, payload):
    length = len(payload)
    cs = checksum(length, cmd, payload)
    return bytes([HEADER, length, cmd, *payload, cs])


def send_set_motor(ser, motor_id, speed, direction):
    speed_u8 = max(0, min(100, int(speed)))
    frame = make_frame(CMD_SET_MOTOR_SPEED, [motor_id, speed_u8, direction])
    ser.write(frame)


def send_set_motor_vel_mrad(ser, motor_id, vel_mrad_s):
    vel_i16 = max(-32768, min(32767, int(round(vel_mrad_s))))
    vel_bytes = struct.pack("<h", vel_i16)
    frame = make_frame(CMD_SET_MOTOR_VEL_MRAD, [motor_id, vel_bytes[0], vel_bytes[1]])
    ser.write(frame)


def send_stop_all(ser):
    frame = make_frame(CMD_STOP_ALL, [])
    ser.write(frame)


def send_enable(ser, motor_id, enable):
    frame = make_frame(CMD_SET_ENABLE, [motor_id, 1 if enable else 0])
    ser.write(frame)


def send_get_status(ser):
    frame = make_frame(CMD_GET_STATUS, [])
    ser.write(frame)


def send_dm_scan_ids(ser):
    frame = make_frame(CMD_DM_SCAN_IDS, [])
    ser.write(frame)


def send_dm_set_id(ser, old_id, new_id):
    frame = make_frame(CMD_DM_SET_ID, [old_id & 0xFF, new_id & 0xFF])
    ser.write(frame)


def send_gateway_control(
    ser, motor_id, enable, p_des, v_des, kp, kd, t_ff,
    target_position=0.0, target_velocity=0.0, control_mode=1, motor_group=1
):
    """Send ARM gateway 39-byte control frame (0xAA 0x55 ... checksum)."""
    frame = bytearray(39)
    frame[0] = 0xAA
    frame[1] = 0x55
    frame[2] = 0x00
    frame[3] = 0x20  # CMD_GATEWAY_CONTROL
    frame[4] = control_mode & 0xFF
    frame[5] = motor_group & 0xFF
    frame[6] = motor_id & 0xFF
    frame[7] = 1 if enable else 0
    frame[8:12] = struct.pack("<f", float(p_des))
    frame[12:16] = struct.pack("<f", float(v_des))
    frame[16:20] = struct.pack("<f", float(kp))
    frame[20:24] = struct.pack("<f", float(kd))
    frame[24:28] = struct.pack("<f", float(t_ff))
    frame[28:32] = struct.pack("<f", float(target_position))
    frame[32:36] = struct.pack("<f", float(target_velocity))
    frame[36] = 0
    frame[37] = 0
    checksum = sum(frame[2:38]) & 0xFF
    frame[38] = checksum
    ser.write(frame)


def send_set_gravity_comp(ser, motor_id, enable, kg, bias_rad, viscous, coulomb):
    payload = bytearray()
    payload.extend([motor_id & 0xFF, 1 if enable else 0])
    payload.extend(struct.pack("<f", float(kg)))
    payload.extend(struct.pack("<f", float(bias_rad)))
    payload.extend(struct.pack("<f", float(viscous)))
    payload.extend(struct.pack("<f", float(coulomb)))
    ser.write(make_frame(CMD_SET_GRAVITY_COMP, payload))


def send_rezero_gravity(ser, motor_id):
    ser.write(make_frame(CMD_REZERO_GRAVITY, [motor_id & 0xFF]))


def send_gripper_jog(ser, dir_code, speed_pct):
    frame = make_frame(CMD_GRIPPER_JOG, [dir_code & 0xFF, max(1, min(100, int(speed_pct)))])
    ser.write(frame)


def send_gripper_set_zero(ser):
    ser.write(make_frame(CMD_GRIPPER_SET_ZERO, []))


def send_gripper_save_cfg(ser):
    ser.write(make_frame(CMD_GRIPPER_SAVE_CFG, []))


def send_gripper_trust_zero(ser):
    ser.write(make_frame(CMD_GRIPPER_TRUST_ZERO, []))


def send_gripper_set_open_pct(ser, open_pct):
    ser.write(make_frame(CMD_GRIPPER_SET_OPEN, [max(0, min(100, int(open_pct)))]))


def send_gripper_get_status(ser):
    ser.write(make_frame(CMD_GRIPPER_GET_STATUS, []))


def force_stop_motor(ser, motor_id):
    """Force-disable motor(s) and send stop frames.

    Always broadcasts to all motors (0xFF) on exit so no motor is left enabled,
    regardless of which one was selected.  Sends each command type twice with a
    short pause to survive one dropped frame on a noisy link.
    """
    all_ids = [1, 2, 3, 4, 5, 0xFF]
    for _ in range(2):
        # Broadcast stop / disable to every motor individually and via 0xFF.
        for mid in all_ids:
            try:
                send_set_motor_vel_mrad(ser, mid, 0)
                send_set_motor(ser, mid, 0, DIR_STOP)
                send_enable(ser, mid, 0)
            except Exception:
                pass
        try:
            send_stop_all(ser)
        except Exception:
            pass
        time.sleep(0.02)
    try:
        ser.flush()
    except Exception:
        pass


def apply_drive_state(ser, motor_id, speed_rad_s, direction):
    if speed_rad_s <= 0.0:
        send_set_motor_vel_mrad(ser, motor_id, 0.0)
        return
    send_enable(ser, motor_id, 1)
    vel_mrad_s = speed_rad_s * 1000.0
    if direction == DIR_BACKWARD:
        vel_mrad_s = -vel_mrad_s
    elif direction == DIR_STOP:
        vel_mrad_s = 0.0
    send_set_motor_vel_mrad(ser, motor_id, vel_mrad_s)


def apply_drive_state_duty(ser, motor_id, duty_pct, direction):
    duty = max(0, min(100, int(round(duty_pct))))
    if duty <= 0:
        send_set_motor(ser, motor_id, 0, DIR_STOP)
        return
    send_enable(ser, motor_id, 1)
    send_set_motor(ser, motor_id, duty, direction)


def ramp_drive_state_duty(ser, motor_id, target_duty_pct, direction, startup_duty_pct, ramp_ms):
    target = max(0.0, min(100.0, float(target_duty_pct)))
    startup = max(1.0, min(100.0, float(startup_duty_pct)))
    ramp_ms = max(0, int(ramp_ms))

    if target <= 0.0:
        apply_drive_state_duty(ser, motor_id, 0.0, direction)
        return

    if ramp_ms == 0 or target <= startup:
        apply_drive_state_duty(ser, motor_id, target, direction)
        return

    steps = max(2, min(20, ramp_ms // 40))
    step_sleep = (ramp_ms / 1000.0) / float(steps)
    for i in range(steps):
        duty = startup + (target - startup) * float(i + 1) / float(steps)
        apply_drive_state_duty(ser, motor_id, duty, direction)
        time.sleep(step_sleep)


def ramp_drive_state_vel(ser, motor_id, target_speed_rad_s, direction, startup_speed_rad_s, ramp_ms):
    target = max(0.0, float(target_speed_rad_s))
    startup = max(0.05, float(startup_speed_rad_s))
    ramp_ms = max(0, int(ramp_ms))

    if target <= 0.0:
        apply_drive_state(ser, motor_id, 0.0, direction)
        return

    if ramp_ms == 0 or target <= startup:
        apply_drive_state(ser, motor_id, target, direction)
        return

    steps = max(2, min(20, ramp_ms // 40))
    step_sleep = (ramp_ms / 1000.0) / float(steps)
    for i in range(steps):
        speed = startup + (target - startup) * float(i + 1) / float(steps)
        apply_drive_state(ser, motor_id, speed, direction)
        time.sleep(step_sleep)


def smooth_stop_drive(ser, motor_id, mode, current_speed_rad_s, current_duty_pct, direction, decel_ms=350):
    """Decelerate to zero before disabling, to avoid abrupt stop."""
    decel_ms = max(80, int(decel_ms))
    steps = max(3, min(24, decel_ms // 35))
    step_sleep = (decel_ms / 1000.0) / float(steps)
    if mode == "duty":
        start = max(0.0, float(current_duty_pct))
        for i in range(steps):
            duty = start * (1.0 - float(i + 1) / float(steps))
            apply_drive_state_duty(ser, motor_id, duty, direction)
            time.sleep(step_sleep)
        apply_drive_state_duty(ser, motor_id, 0.0, direction)
    else:
        start = max(0.0, float(current_speed_rad_s))
        for i in range(steps):
            speed = start * (1.0 - float(i + 1) / float(steps))
            apply_drive_state(ser, motor_id, speed, direction)
            time.sleep(step_sleep)
        apply_drive_state(ser, motor_id, 0.0, direction)


class FrameParser:
    def __init__(self):
        self.reset()

    def reset(self):
        self.state = "header"
        self.length = 0
        self.cmd = 0
        self.payload = bytearray()
        self.calc = 0

    def feed_byte(self, b):
        if self.state == "header":
            if b == HEADER:
                self.state = "length"
            return None

        if self.state == "length":
            self.length = b
            if self.length > 64:
                self.reset()
                return None
            self.calc = b
            self.state = "cmd"
            return None

        if self.state == "cmd":
            self.cmd = b
            self.calc = (self.calc + b) & 0xFF
            self.payload = bytearray()
            if self.length == 0:
                self.state = "checksum"
            else:
                self.state = "data"
            return None

        if self.state == "data":
            self.payload.append(b)
            self.calc = (self.calc + b) & 0xFF
            if len(self.payload) >= self.length:
                self.state = "checksum"
            return None

        if self.state == "checksum":
            ok = (b == self.calc)
            out = (self.cmd, bytes(self.payload)) if ok else None
            self.reset()
            return out

        self.reset()
        return None


def decode_status_reply(payload):
    if len(payload) < 16:
        return None
    motor_group = payload[0]
    control_mode = payload[1]
    motor_id = payload[2]
    enabled = payload[3]
    position = struct.unpack("<f", payload[4:8])[0]
    velocity = struct.unpack("<f", payload[8:12])[0]
    effort = struct.unpack("<f", payload[12:16])[0]
    if len(payload) >= 17:
        status = payload[16]
    else:
        # Backward compatibility with older firmware that packed status at byte 15
        # (and overwrote tau's last byte).
        status = payload[15]
    return {
        "group": motor_group,
        "mode": control_mode,
        "id": motor_id,
        "enabled": enabled,
        "pos": position,
        "vel": velocity,
        "tau": effort,
        "status": status,
    }


def decode_gripper_status(payload):
    if len(payload) < 24:
        return None
    return {
        "state": payload[0],
        "zero_valid": payload[1],
        "pos": struct.unpack("<i", payload[2:6])[0],
        "target": struct.unpack("<i", payload[6:10])[0],
        "min": struct.unpack("<i", payload[10:14])[0],
        "max": struct.unpack("<i", payload[14:18])[0],
        "margin": struct.unpack("<H", payload[18:20])[0],
        "speed": struct.unpack("<H", payload[20:22])[0],
    }


def gripper_state_name(state):
    if state == 0:
        return "UNHOMED"
    if state == 1:
        return "READY"
    if state == 2:
        return "FAULT"
    return f"UNK({state})"


def poll_device_messages(ser, parser, budget_s=0.01, feedback_cb=None, gripper_cb=None):
    end_t = time.time() + budget_s
    status_frame = None
    while time.time() < end_t:
        try:
            waiting = getattr(ser, "in_waiting", 0)
            if waiting <= 0:
                break
            raw = ser.read(waiting)
        except serial.SerialException:
            raise   # let the caller handle reconnection
        if not raw:
            break
        for b in raw:
            frame = parser.feed_byte(b)
            if frame is not None:
                cmd, payload = frame
                if cmd == CMD_STATUS_REPLY:
                    status_frame = decode_status_reply(payload)
                    if status_frame is not None:
                        if feedback_cb is not None:
                            feedback_cb(status_frame)
                elif cmd == CMD_DM_SCAN_IDS_REPLY and payload:
                    cnt = payload[0]
                    ids = list(payload[1 : 1 + cnt])
                    print(f"\r\033[K[SCAN] count={cnt}, ids={ids}")
                elif cmd == CMD_DM_SET_ID_REPLY and len(payload) >= 4:
                    print(
                        f"\r\033[K[SET_ID] old={payload[0]} new={payload[1]} ok={payload[2]} detail={payload[3]}"
                    )
                elif cmd == CMD_GRIPPER_STATUS_REPLY:
                    gs = decode_gripper_status(payload)
                    if gs is not None:
                        if gripper_cb is not None:
                            gripper_cb(gs)
                        print(
                            f"\r\033[K[GRIPPER] state={gs['state']} zero={gs['zero_valid']} "
                            f"pos={gs['pos']} tgt={gs['target']} lim=[{gs['min']},{gs['max']}] "
                            f"margin={gs['margin']} speed={gs['speed']} step/s"
                        )
                else:
                    print(f"\r\033[K[FRAME] cmd=0x{cmd:02X} payload={list(payload)}")
    return status_frame


def set_enable_with_verify(ser, parser, motor_id, enable, retries=3, timeout_s=0.35, feedback_cb=None):
    """Send enable/disable and verify by status frame when possible."""
    target = 1 if enable else 0
    for _ in range(max(1, retries)):
        send_enable(ser, motor_id, target)
        send_get_status(ser)
        t_end = time.time() + timeout_s
        while time.time() < t_end:
            st = poll_device_messages(ser, parser, budget_s=0.03, feedback_cb=feedback_cb)
            if st is None:
                continue
            if motor_id == 0xFF:
                # For ALL motors we accept any returned frame; firmware reports one focus motor at a time.
                if int(st.get("enabled", 0)) == target:
                    return True
            elif int(st.get("id", -1)) == int(motor_id):
                if int(st.get("enabled", 0)) == target:
                    return True
        time.sleep(0.03)
    return False


def detect_ttyusb_port(preferred=None):
    if preferred and preferred.startswith("socket://"):
        return preferred

    if preferred and os.path.exists(preferred):
        return preferred

    ports = sorted(glob.glob("/dev/ttyUSB*"))
    if not ports:
        return None
    return ports[0]


def try_fix_permission(port):
    try:
        subprocess.run(["sudo", "chmod", "666", port], check=False)
    except Exception:
        pass


def open_serial_with_recover(port, baud, recover_timeout):
    if isinstance(port, str) and port.startswith("socket://"):
        return serial.serial_for_url(port, baudrate=baud, timeout=0.1), port

    start = time.time()
    last_error = ""

    while True:
        active_port = detect_ttyusb_port(port)
        if active_port is None:
            last_error = "Serial port not found."
        else:
            try:
                s = serial.Serial(active_port, baud, timeout=0.1, dsrdtr=False, rtscts=False)
                s.setDTR(False)  # DTR=0 → NRST released (no reset)
                s.setRTS(False)  # RTS=0 → BOOT0=0 (application mode, not bootloader)
                return s, active_port
            except Exception as exc:
                last_error = str(exc)
                if "Permission denied" in last_error:
                    try_fix_permission(active_port)
                    try:
                        s = serial.Serial(active_port, baud, timeout=0.1, dsrdtr=False, rtscts=False)
                        s.setDTR(False)
                        s.setRTS(False)
                        return s, active_port
                    except Exception as exc2:
                        last_error = str(exc2)

        if recover_timeout <= 0:
            raise RuntimeError(last_error)
        if time.time() - start > recover_timeout:
            raise RuntimeError(last_error)
        time.sleep(0.6)


def print_status(
    speed_rad_s,
    direction,
    mode=None,
    duty_pct=None,
    enabled=True,
    selected_motor=1,
    kp=None,
    kd=None,
    kg=None,
    bias=None,
    grav_en=None,
    last_fb=None,
    gripper_fb=None,
    zero_pos_rad=None,    # θ at last rezero (C key), used for model diagnostics
    grav_calibrated=False, # True after C pressed at horizontal
):
    dir_text = "FWD" if direction == DIR_FORWARD else "BWD"
    en_text = "EN " if enabled else "DIS"
    motor_text = "ALL" if selected_motor == 0xFF else f"M{selected_motor}"

    if mode == "duty":
        cmd_text = f"Duty:{0 if duty_pct is None else int(round(duty_pct)):3d}%"
    else:
        cmd_text = f"Spd:{speed_rad_s:5.2f}r/s"

    tune_text = ""
    if kp is not None and kd is not None and kg is not None and grav_en is not None:
        bias_deg = (bias or 0.0) * 57.2958
        cal_tag = "CAL" if grav_calibrated else "!NC"   # !NC = Not Calibrated
        tune_text = (
            f" KP:{kp:4.1f} KD:{kd:4.2f}"
            f" KG:{kg:4.2f} bias:{bias_deg:+6.1f}°"
            f" G:{'Y' if grav_en else 'N'}[{cal_tag}]"
        )

    fb_text = ""
    if last_fb is not None:
        pos_r        = last_fb['pos']
        # theta_local = pos - zero_pos (mirrors firmware computation).
        # This is the angle that matters for gravity compensation.
        theta_local  = pos_r - (zero_pos_rad or 0.0)
        theta_loc_deg = theta_local * 57.2958
        bias_r       = bias or 0.0
        model_ff     = (kg or 0.0) * math.cos(theta_local + bias_r)
        fb_text = (
            f" | θ:{theta_loc_deg:+6.1f}°"           # angle from gravity zero (B key)
            f" v:{last_fb['vel']:+5.2f}r/s"
            f" t:{last_fb['tau']:+5.2f}Nm"
            f" ff:{model_ff:+5.2f}Nm"
            f" st:{last_fb['status']}"
        )

    gripper_text = ""
    if gripper_fb is not None:
        gripper_text = (
            f" | G42:{gripper_state_name(gripper_fb['state'])}"
            f" z:{gripper_fb['zero_valid']}"
            f" pos*:{gripper_fb['pos']}"
            f" tgt:{gripper_fb['target']}"
        )

    print(
        f"\r\033[K[{motor_text}] {en_text} {cmd_text} {dir_text}{tune_text}{fb_text}{gripper_text}",
        end="",
        flush=True,
    )


def main():
    parser = argparse.ArgumentParser(description="Keyboard control STM32 motor via serial")
    parser.add_argument("--port", default="/dev/ttyUSB0", help="Serial port, e.g. /dev/ttyUSB0")
    parser.add_argument("--socket-host", default=None, help="Use TCP serial bridge host, e.g. 192.168.10.2")
    parser.add_argument("--socket-port", type=int, default=8888, help="TCP serial bridge port")
    parser.add_argument("--baud", type=int, default=115200, help="Baudrate")
    parser.add_argument("--motor", type=int, default=1, choices=[1, 2, 3, 4, 5, 255], help="Motor id (1-5, 255=all)")
    parser.add_argument("--mode", choices=["duty", "vel"], default="vel", help="Control mode for A/D")
    parser.add_argument("--step", type=int, default=5, help="Hold duty percent for A/D in duty mode")
    parser.add_argument("--speed-rad-s", type=float, default=0.40, help="Hold speed for A/D in rad/s (safe default: 0.40)")
    parser.add_argument("--startup-duty", type=float, default=5.0, help="Duty used at startup ramp in duty mode")
    parser.add_argument("--startup-speed-rad-s", type=float, default=0.15, help="Speed used at startup ramp in vel mode")
    parser.add_argument("--ramp-ms", type=int, default=1200, help="Startup ramp duration in milliseconds (0 to disable)")
    parser.add_argument("--recover-timeout", type=int, default=30, help="Seconds to wait/recover serial after replug (0 to disable)")
    parser.add_argument("--refresh-ms", type=int, default=120, help="Resend current command periodically (ms)")
    parser.add_argument("--hold-timeout-ms", type=int, default=220, help="Release timeout for A/D hold (ms), 0 to disable")
    parser.add_argument("--decel-ms", type=int, default=350, help="Smooth deceleration duration when stop/timeout")
    parser.add_argument("--status-period-ms", type=int, default=200, help="Polling period for CMD_GET_STATUS (0 disable)")
    parser.add_argument("--gripper-status-period-ms", type=int, default=300, help="Polling period for CMD_GRIPPER_GET_STATUS (0 disable)")
    parser.add_argument("--comm-test", action="store_true", help="Only test link/status then exit")
    parser.add_argument("--log-csv", default="", help="Path to CSV for feedback logging (default auto timestamp if enabled)")
    parser.add_argument("--no-log", action="store_true", help="Disable feedback CSV logging")
    parser.add_argument("--kp-step", type=float, default=0.5, help="Online tuning step for KP")
    parser.add_argument("--kd-step", type=float, default=0.05, help="Online tuning step for KD")
    parser.add_argument("--kg-step", type=float, default=0.02, help="Online tuning step for gravity gain KG")
    parser.add_argument("--bias-step", type=float, default=0.005, help="Online tuning step for gravity bias (rad)")
    parser.add_argument("--scan-ids", action="store_true", help="Scan DM IDs once, print and exit")
    parser.add_argument("--set-id", nargs=2, type=int, metavar=("OLD_ID", "NEW_ID"), help="Set motor id then exit")
    parser.add_argument("--gripper-open-pct", type=int, default=None, help="Set gripper opening percent [0..100] then exit")
    parser.add_argument("--gripper-trust-zero", action="store_true", help="Trust saved gripper zero then exit")
    parser.add_argument("--gripper-set-zero", action="store_true", help="Set current gripper as zero then exit")
    parser.add_argument("--gripper-save-cfg", action="store_true", help="Save gripper config to flash then exit")
    args = parser.parse_args()

    if args.socket_host:
        args.port = f"socket://{args.socket_host}:{args.socket_port}"

    speed_rad_s = 0.0
    duty_pct = 0.0
    direction = DIR_FORWARD
    if args.speed_rad_s is not None:
        hold_speed_rad_s = max(0.05, min(1.2, args.speed_rad_s))
    else:
        hold_speed_pct = max(1, min(100, args.step))
        hold_speed_rad_s = (hold_speed_pct * 1.2) / 100.0

    actual_hold_rad_s = hold_speed_rad_s
    refresh_s = max(0.03, args.refresh_ms / 1000.0)
    hold_timeout_s = None
    if args.hold_timeout_ms > 0:
        hold_timeout_s = max(0.05, args.hold_timeout_ms / 1000.0)
    drive_active = False
    motor_enabled = True
    last_drive_key_time = 0.0

    hold_duty_pct = float(max(1, min(100, args.step)))

    print("Keyboard control started")
    print("[SAFE] Motors are enabled first, then command follows keys.")
    print("[SAFE] Default speed is intentionally slow (0.40 rad/s, capped <=1.2 rad/s).")
    print("[SAFE] Release timeout auto-stop is enabled.")
    if args.mode == "duty":
        print("Controls: A backward, D forward, W/S adjust duty, Space stop, Q quit")
        print(f"Hold duty: {hold_duty_pct:.0f}%")
    else:
        print("Controls: A backward, D forward, W/S adjust speed, Space stop, Q quit")
        print(f"Hold speed: {actual_hold_rad_s:.2f} rad/s")
    print("Extra: 1~5 select motor, 0 select all, R request status, X scan IDs")
    print("Gripper: J close, L open, K stop, U query, Z set-zero, V save, T trust-zero, O/P open%-/+")
    print("Tune: [/] KP-/+, ;/' KD-/+, ,/. KG-/+, N/M bias-/+, F enable, G gravity on/off, B flat-zero")
    print("Tune: H=zero KP&KD (float)  C=set horizontal here  E=latch pos  Y=reset bias")
    print("ID map: ID5->joint1(M24 yaw-z), ID1->joint2(DM4310), ID2->joint3, ID3->joint4, ID4->joint6")
    print("── KG tuning: H float → move to HORIZONTAL → C calibrate → ,/. KG until hover → E latch → ] KP ──")

    try:
        ser, real_port = open_serial_with_recover(args.port, args.baud, args.recover_timeout)
        ser.dtr = False
        ser.rts = False
        print(f"Using serial port: {real_port}")
    except Exception as exc:
        print(f"Open serial failed: {exc}")
        print("Try: sudo chmod 666 /dev/ttyUSB0")
        return 1

    selected_motor = args.motor
    parser_state = FrameParser()
    status_period_s = args.status_period_ms / 1000.0 if args.status_period_ms > 0 else None
    gripper_status_period_s = args.gripper_status_period_ms / 1000.0 if args.gripper_status_period_ms > 0 else None
    log_fp = None
    log_writer = None
    last_gripper = {"frame": None}

    tune_state = {
        mid: {
            "kp": 0.0, "kd": 0.0, "kg": 0.0, "bias": 0.0,
            "viscous": 0.0, "coulomb": 0.0, "grav_en": True,
            "_zero_pos_rad": 0.0,     # θ when last rezero was issued (tracks firmware zero)
            "_grav_calibrated": False, # True only after C key pressed at horizontal
        }
        for mid in [1, 2, 3, 4, 5]
    }
    # ── Per-motor defaults derived from 三款电机参数对比表.xlsx ──────────────────
    #
    # DM4310  id=1  base_pitch   peak 7.0 N.m  10:1 reduction  120 rpm rated
    #   KP safety: peak / 0.5 rad ≈ 14  →  use 12 (conservative)
    #   KD: 14-pole, moderate inertia → 1.5
    tune_state[1]["kp"]      = 12.0
    tune_state[1]["kd"]      =  1.5
    tune_state[1]["kg"]      =  0.55   # empirical gravity comp; tune with ,/.
    tune_state[1]["grav_en"] = True
    #
    # DH3510  id=2/3/4  arm joints  peak 0.45 N.m  1:1 DIRECT DRIVE  500 rpm
    #   KP safety: peak / 0.15 rad = 3.0  →  use 3  (>0.15 rad error saturates motor)
    #   KD: direct drive, low inductance (1.67 μH) → small, else noisy
    tune_state[2]["kp"]      =  3.0
    tune_state[2]["kd"]      =  0.3
    tune_state[2]["kg"]      =  0.40   # arm segment weight at joint 3
    tune_state[2]["grav_en"] = True

    tune_state[3]["kp"]      =  3.0
    tune_state[3]["kd"]      =  0.3
    tune_state[3]["kg"]      =  0.22   # lighter load further down the arm
    tune_state[3]["grav_en"] = True

    tune_state[4]["kp"]      =  3.0
    tune_state[4]["kd"]      =  0.3
    tune_state[4]["kg"]      =  0.12   # wrist / gripper carrier, minimal gravity
    tune_state[4]["grav_en"] = True
    #
    # M24  id=5  base_yaw_Z  peak 23.5 N.m  6:1 reduction  140 rpm rated
    #   KP safety: peak / 0.5 rad = 47  →  use 20 (comfortable margin)
    #   Z-axis rotation → gravity term ≈ 0
    tune_state[5]["kp"]      = 20.0
    tune_state[5]["kd"]      =  2.0
    tune_state[5]["kg"]      =  0.00
    tune_state[5]["grav_en"] = False   # no gravity comp for yaw axis

    if args.scan_ids:
        with ser:
            send_dm_scan_ids(ser)
            t_end = time.time() + 2.0
            while time.time() < t_end:
                poll_device_messages(ser, parser_state, budget_s=0.05)
                time.sleep(0.02)
        print("Scan done.")
        return 0

    if args.set_id is not None:
        old_id, new_id = args.set_id
        with ser:
            send_dm_set_id(ser, old_id, new_id)
            t_end = time.time() + 2.0
            while time.time() < t_end:
                poll_device_messages(ser, parser_state, budget_s=0.05)
                time.sleep(0.02)
        print("Set id done.")
        return 0

    if args.gripper_trust_zero or args.gripper_set_zero or args.gripper_save_cfg or args.gripper_open_pct is not None:
        with ser:
            if args.gripper_trust_zero:
                send_gripper_trust_zero(ser)
            if args.gripper_set_zero:
                send_gripper_set_zero(ser)
            if args.gripper_save_cfg:
                send_gripper_save_cfg(ser)
            if args.gripper_open_pct is not None:
                send_gripper_set_open_pct(ser, args.gripper_open_pct)
            send_gripper_get_status(ser)
            t_end = time.time() + 1.2
            while time.time() < t_end:
                poll_device_messages(ser, parser_state, budget_s=0.05)
                time.sleep(0.02)
        print("Gripper command done.")
        return 0

    if args.comm_test:
        with ser:
            print("[COMM] Sending status requests ...")
            ok = False
            for _ in range(6):
                send_get_status(ser)
                st = poll_device_messages(ser, parser_state, budget_s=0.15)
                if st is not None:
                    ok = True
                    break
                time.sleep(0.05)
            if ok:
                print("[COMM] OK: status frame received.")
                return 0
            print("[COMM] FAIL: no status frame. Check socat/serial/baud/port.")
            return 2

    if not args.no_log:
        log_path = args.log_csv.strip()
        if not log_path:
            stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            log_path = f"motor_feedback_{stamp}.csv"
        log_fp = open(log_path, "w", newline="", encoding="utf-8")
        log_writer = csv.writer(log_fp)
        log_writer.writerow(["ts", "motor_id", "joint", "motor_type", "enabled", "pos_rad", "vel_rad_s", "tau_nm", "status"])
        print(f"[LOG] Feedback CSV: {log_path}")

    last_fb_by_motor = {}

    def on_feedback(frame):
        last_fb_by_motor[int(frame["id"])] = frame
        if log_writer is not None:
            log_writer.writerow([
                f"{time.time():.6f}",
                int(frame["id"]),
                MOTOR_JOINT_MAP.get(int(frame["id"]), {}).get("joint", "unknown"),
                MOTOR_JOINT_MAP.get(int(frame["id"]), {}).get("type", "unknown"),
                int(frame["enabled"]),
                f"{frame['pos']:.6f}",
                f"{frame['vel']:.6f}",
                f"{frame['tau']:.6f}",
                int(frame["status"]),
            ])
            if log_fp is not None:
                log_fp.flush()

    def on_gripper_feedback(frame):
        last_gripper["frame"] = frame

    stop_done = {"value": False}

    def ensure_stopped():
        if stop_done["value"]:
            return
        stop_done["value"] = True
        try:
            force_stop_motor(ser, selected_motor)
        except Exception:
            pass

    def _signal_stop_handler(signum, _frame):
        ensure_stopped()
        raise SystemExit(128 + int(signum))

    def current_tune(mid):
        if mid == 0xFF:
            return tune_state[1]
        return tune_state.get(mid, tune_state[1])

    def do_rezero(mid):
        """Send rezero and snapshot current position as the new gravity zero."""
        send_rezero_gravity(ser, mid)
        mids = [1, 2, 3, 4, 5] if mid == 0xFF else [mid]
        for i in mids:
            fb = last_fb_by_motor.get(i)
            if fb is not None:
                tune_state[i]["_zero_pos_rad"] = fb["pos"]

    def push_online_tune(mid, enable=None):
        """Push KP/KD/KG/bias to firmware.

        The `enable` flag in the gateway control frame is taken from the current
        `motor_enabled` state unless explicitly overridden.  This prevents
        tuning keystrokes (,  .  [  ]  etc.) from silently re-enabling the motor
        while Python considers it disabled.
        """
        def _push_one(i):
            ts = tune_state[i]
            latched = ts.get("_latched_pos")
            fb = last_fb_by_motor.get(i)
            p_cur = latched if latched is not None else (fb["pos"] if fb is not None else 0.0)
            eff_en = motor_enabled if enable is None else bool(enable)
            send_gateway_control(ser, i, eff_en, p_cur, 0.0, ts["kp"], ts["kd"], 0.0)
            send_set_gravity_comp(
                ser, i, ts["grav_en"], ts["kg"], ts["bias"], ts["viscous"], ts["coulomb"]
            )

        if mid == 0xFF:
            for i in [1, 2, 3, 4, 5]:
                _push_one(i)
            return
        _push_one(mid)

    atexit.register(ensure_stopped)
    signal.signal(signal.SIGINT, _signal_stop_handler)
    signal.signal(signal.SIGTERM, _signal_stop_handler)

    def _reconnect_ser():
        """Close the dead connection and open a fresh one.  Returns True on success."""
        nonlocal ser
        print(f"\r\n[DISCONNECT] socket lost – reconnecting to {args.port} …", flush=True)
        try:
            ser.close()
        except Exception:
            pass
        for attempt in range(1, 6):
            time.sleep(2.0)
            try:
                new_ser, real_port = open_serial_with_recover(args.port, args.baud, 8)
                new_ser.setDTR(False)
                new_ser.setRTS(False)
                time.sleep(0.5)
                new_ser.flushInput()
                ser = new_ser
                print(f"\r\n[RECONNECT] OK ({real_port}) attempt {attempt}", flush=True)
                return True
            except Exception as exc2:
                print(f"\r\n[RECONNECT] attempt {attempt}/5 failed: {exc2}", flush=True)
        print("\r\n[RECONNECT] Giving up after 5 attempts.", flush=True)
        return False

    with RawTerminal():
        try:
            # Motors start DISABLED for safety. Press F to enable.
            motor_enabled = False
            # Wait for STM32 to finish booting after any DTR/RTS-induced reset.
            time.sleep(0.8)
            ser.flushInput()
            print("\r\033[K[INIT] Motors disabled. Press F to enable, then move to horizontal and press C.", flush=True)
            ts = current_tune(selected_motor)
            fb = last_fb_by_motor.get(selected_motor)
            fb_key = selected_motor if selected_motor != 0xFF else 1
            cal = tune_state[fb_key]["_grav_calibrated"]
            print_status(
                speed_rad_s, direction, args.mode, duty_pct, motor_enabled, selected_motor,
                ts["kp"], ts["kd"], ts["kg"], bias=ts["bias"],
                grav_en=ts["grav_en"], last_fb=fb,
                zero_pos_rad=ts["_zero_pos_rad"],
                grav_calibrated=cal,
            )
            last_refresh = time.time()
            last_status = 0.0
            last_gripper_status = 0.0
            gripper_open_pct = 50
            while True:
                rlist, _, _ = select.select([sys.stdin], [], [], 0.05)
                ch = sys.stdin.read(1) if rlist else ""

                if ch:
                    c = ch.lower()
                    if c == "q":
                        smooth_stop_drive(
                            ser, selected_motor, args.mode, speed_rad_s, duty_pct, direction, args.decel_ms
                        )
                        speed_rad_s = 0.0
                        duty_pct = 0.0
                        motor_enabled = False
                        ensure_stopped()
                        print("\nQuit and stop all motors.")
                        break

                    if c in ["1", "2", "3", "4", "5"]:
                        if drive_active:
                            smooth_stop_drive(ser, selected_motor, args.mode, speed_rad_s, duty_pct, direction, 150)
                        selected_motor = int(c)
                        motor_enabled = False
                        drive_active = False
                        speed_rad_s = 0.0
                        duty_pct = 0.0
                        mj = MOTOR_JOINT_MAP.get(selected_motor, {})
                        print(f"\r\033[K[SELECT] M{selected_motor} {mj.get('type','?')} -> {mj.get('joint','?')}  (DIS, press F to enable)")
                    elif c == "0":
                        if drive_active:
                            smooth_stop_drive(ser, selected_motor, args.mode, speed_rad_s, duty_pct, direction, 150)
                        selected_motor = 0xFF
                        motor_enabled = False
                        drive_active = False
                        speed_rad_s = 0.0
                        duty_pct = 0.0
                        print(f"\r\033[K[SELECT] ALL motors  (DIS, press F to enable)")
                    elif c == "r":
                        send_get_status(ser)
                    elif c == "x":
                        send_dm_scan_ids(ser)
                    elif c == "j":
                        send_gripper_jog(ser, 0, 25)
                    elif c == "l":
                        send_gripper_jog(ser, 1, 25)
                    elif c == "k":
                        send_gripper_jog(ser, 2, 25)
                    elif c == "u":
                        send_gripper_get_status(ser)
                    elif c == "z":
                        send_gripper_set_zero(ser)
                    elif c == "v":
                        send_gripper_save_cfg(ser)
                    elif c == "t":
                        send_gripper_trust_zero(ser)
                    elif c == "o":
                        gripper_open_pct = max(0, gripper_open_pct - 5)
                        send_gripper_set_open_pct(ser, gripper_open_pct)
                    elif c == "p":
                        gripper_open_pct = min(100, gripper_open_pct + 5)
                        send_gripper_set_open_pct(ser, gripper_open_pct)
                    elif c == "f":
                        if motor_enabled:
                            # Disable: smooth-stop first, then disable all motors.
                            smooth_stop_drive(
                                ser, selected_motor, args.mode, speed_rad_s, duty_pct, direction, args.decel_ms
                            )
                            drive_active = False
                            speed_rad_s = 0.0
                            duty_pct = 0.0
                            send_enable(ser, selected_motor, 0)
                            motor_enabled = False
                        else:
                            # Enable: push correct KP/KD/KG via gateway control.
                            # Do NOT rezero here – zero_pos_rad is only valid once
                            # the user moves the arm to horizontal and presses C.
                            # Until C is pressed, firmware returns 0 gravity ff.
                            push_online_tune(selected_motor, enable=True)
                            motor_enabled = True
                            print(f"\r\033[K[ENABLE] Motor enabled. Move arm to HORIZONTAL, then press C to activate gravity comp.")
                    elif c == "g":
                        ts = current_tune(selected_motor)
                        ts["grav_en"] = not ts["grav_en"]
                        push_online_tune(selected_motor)
                    elif c == "b":
                        # B = set gravity zero at current position.
                        # MUST be pressed when arm is exactly horizontal.
                        do_rezero(selected_motor)
                    elif c == "c":
                        # C = calibrate horizontal zero.
                        # Move the arm to TRUE horizontal first, THEN press C.
                        # This sets zero_pos_rad to the current position so that
                        # theta_local=0 (cos peak = 1) is exactly at horizontal.
                        # Gravity comp activates immediately after this press.
                        mids = [1, 2, 3, 4, 5] if selected_motor == 0xFF else [selected_motor]
                        for mid in mids:
                            fb = last_fb_by_motor.get(mid)
                            if fb is None:
                                continue
                            ts_c = tune_state[mid]
                            ts_c["_zero_pos_rad"] = fb["pos"]
                            ts_c["_grav_calibrated"] = True
                        do_rezero(selected_motor)
                        print(f"\r\033[K[CAL] Horizontal zero calibrated – gravity comp now active. Tune KG with </>.")
                    elif c == "y":
                        # Reset bias to 0.  The firmware now uses cos(θ_local + bias)
                        # natively, so bias = 0 is the correct baseline.
                        # Use N/M only for fine-trimming residual zero-offset.
                        mids = [1, 2, 3, 4, 5] if selected_motor == 0xFF else [selected_motor]
                        for mid in mids:
                            tune_state[mid]["bias"] = 0.0
                        push_online_tune(selected_motor)
                        print(f"\r\033[K[BIAS] reset to 0° – firmware uses cos(θ_local) natively")
                    elif c == "[":
                        ts = current_tune(selected_motor)
                        ts["kp"] = max(0.0, ts["kp"] - args.kp_step)
                        push_online_tune(selected_motor)
                    elif c == "]":
                        ts = current_tune(selected_motor)
                        ts["kp"] = min(80.0, ts["kp"] + args.kp_step)
                        push_online_tune(selected_motor)
                    elif c == ";":
                        ts = current_tune(selected_motor)
                        ts["kd"] = max(0.0, ts["kd"] - args.kd_step)
                        push_online_tune(selected_motor)
                    elif c == "'":
                        ts = current_tune(selected_motor)
                        ts["kd"] = min(15.0, ts["kd"] + args.kd_step)
                        push_online_tune(selected_motor)
                    elif c == ",":
                        ts = current_tune(selected_motor)
                        ts["kg"] = max(-8.0, ts["kg"] - args.kg_step)
                        push_online_tune(selected_motor)
                    elif c == ".":
                        ts = current_tune(selected_motor)
                        ts["kg"] = min(8.0, ts["kg"] + args.kg_step)
                        push_online_tune(selected_motor)
                    elif c == "n":
                        ts = current_tune(selected_motor)
                        ts["bias"] = max(-3.2, ts["bias"] - args.bias_step)
                        push_online_tune(selected_motor)
                    elif c == "m":
                        ts = current_tune(selected_motor)
                        ts["bias"] = min(3.2, ts["bias"] + args.bias_step)
                        push_online_tune(selected_motor)
                    elif c == "h":
                        # ── Gravity-only float mode ──────────────────────────────
                        # Zero KP and KD so the motor is held up ONLY by gravity
                        # compensation feedforward.  Use this to tune KG:
                        #   1. Press H → arm goes limp (gravity comp only)
                        #   2. Press ,/. to adjust KG until arm hovers at any angle
                        #   3. Press E to latch current position, then ]  to raise KP
                        mids = [1, 2, 3, 4, 5] if selected_motor == 0xFF else [selected_motor]
                        for mid in mids:
                            tune_state[mid]["kp"] = 0.0
                            tune_state[mid]["kd"] = 0.0
                        push_online_tune(selected_motor)
                        print(f"\r\033[K[FLOAT] KP=0 KD=0 – gravity comp only. Tune KG with ,/.  then press E to latch pos.")
                    elif c == "e":
                        # ── Latch current position as hold target ────────────────
                        # Snapshot the latest feedback position as p_des, then push
                        # the current KP/KD.  Use after KG is tuned to test position
                        # hold: slowly raise KP with ] until arm stops drifting.
                        mids = [1, 2, 3, 4, 5] if selected_motor == 0xFF else [selected_motor]
                        for mid in mids:
                            fb = last_fb_by_motor.get(mid)
                            if fb is not None:
                                tune_state[mid]["_latched_pos"] = fb["pos"]
                        push_online_tune(selected_motor)
                        latched = {mid: last_fb_by_motor[mid]["pos"] * 57.2958
                                   for mid in mids if mid in last_fb_by_motor}
                        print(f"\r\033[K[LATCH] target={latched}°  now raise KP with ]")

                    if c == "a":
                        if not motor_enabled:
                            print(f"\r\033[K[WARN] Motor DIS -- press F to enable first.")
                        else:
                            if direction != DIR_BACKWARD:
                                if args.mode == "duty":
                                    send_set_motor(ser, selected_motor, 0, DIR_STOP)
                                else:
                                    send_set_motor_vel_mrad(ser, selected_motor, 0.0)
                                time.sleep(0.05)
                            direction = DIR_BACKWARD
                            speed_rad_s = hold_speed_rad_s
                            duty_pct = hold_duty_pct
                            drive_active = True
                            last_drive_key_time = time.time()
                            if args.mode == "duty":
                                ramp_drive_state_duty(
                                    ser, selected_motor, duty_pct, direction,
                                    args.startup_duty, args.ramp_ms,
                                )
                            else:
                                ramp_drive_state_vel(
                                    ser, selected_motor, speed_rad_s, direction,
                                    args.startup_speed_rad_s, args.ramp_ms,
                                )
                    elif c == "d":
                        if not motor_enabled:
                            print(f"\r\033[K[WARN] Motor DIS -- press F to enable first.")
                        else:
                            if direction != DIR_FORWARD:
                                if args.mode == "duty":
                                    send_set_motor(ser, selected_motor, 0, DIR_STOP)
                                else:
                                    send_set_motor_vel_mrad(ser, selected_motor, 0.0)
                                time.sleep(0.05)
                            direction = DIR_FORWARD
                            speed_rad_s = hold_speed_rad_s
                            duty_pct = hold_duty_pct
                            drive_active = True
                            last_drive_key_time = time.time()
                            if args.mode == "duty":
                                ramp_drive_state_duty(
                                    ser, selected_motor, duty_pct, direction,
                                    args.startup_duty, args.ramp_ms,
                                )
                            else:
                                ramp_drive_state_vel(
                                    ser, selected_motor, speed_rad_s, direction,
                                    args.startup_speed_rad_s, args.ramp_ms,
                                )
                    elif c == " ":
                        smooth_stop_drive(
                            ser, selected_motor, args.mode, speed_rad_s, duty_pct, direction, args.decel_ms
                        )
                        speed_rad_s = 0.0
                        duty_pct = 0.0
                        drive_active = False
                        motor_enabled = False
                    elif c == "w":
                        if args.mode == "duty":
                            hold_duty_pct = min(100.0, hold_duty_pct + 2.0)
                            if drive_active:
                                duty_pct = hold_duty_pct
                                apply_drive_state_duty(ser, selected_motor, duty_pct, direction)
                        else:
                            hold_speed_rad_s = min(VEL_MAX_RAD_S, hold_speed_rad_s + 0.1)
                            if drive_active:
                                speed_rad_s = hold_speed_rad_s
                                apply_drive_state(ser, selected_motor, speed_rad_s, direction)
                    elif c == "s":
                        if args.mode == "duty":
                            hold_duty_pct = max(1.0, hold_duty_pct - 2.0)
                            if drive_active:
                                duty_pct = hold_duty_pct
                                apply_drive_state_duty(ser, selected_motor, duty_pct, direction)
                        else:
                            hold_speed_rad_s = max(0.05, hold_speed_rad_s - 0.1)
                            if drive_active:
                                speed_rad_s = hold_speed_rad_s
                                apply_drive_state(ser, selected_motor, speed_rad_s, direction)

                    last_refresh = time.time()

                if hold_timeout_s is not None and drive_active and (time.time() - last_drive_key_time) > hold_timeout_s:
                    drive_active = False
                    if speed_rad_s != 0.0 or duty_pct != 0.0:
                        smooth_stop_drive(
                            ser, selected_motor, args.mode, speed_rad_s, duty_pct, direction, args.decel_ms
                        )
                        speed_rad_s = 0.0
                        duty_pct = 0.0

                if drive_active and speed_rad_s > 0.0 and (time.time() - last_refresh) >= refresh_s:
                    motor_enabled = True
                    if args.mode == "duty":
                        apply_drive_state_duty(ser, selected_motor, duty_pct, direction)
                    else:
                        apply_drive_state(ser, selected_motor, speed_rad_s, direction)
                    last_refresh = time.time()

                try:
                    if status_period_s is not None and (time.time() - last_status) >= status_period_s:
                        send_get_status(ser)
                        last_status = time.time()

                    if gripper_status_period_s is not None and (time.time() - last_gripper_status) >= gripper_status_period_s:
                        send_gripper_get_status(ser)
                        last_gripper_status = time.time()

                    poll_device_messages(ser, parser_state, feedback_cb=on_feedback, gripper_cb=on_gripper_feedback)
                except serial.SerialException as _exc:
                    # Socket/serial disconnected – attempt reconnect.
                    if not _reconnect_ser():
                        break
                    # Re-push tune parameters so the motor resumes its last state.
                    parser_state.reset()
                    if motor_enabled:
                        try:
                            push_online_tune(selected_motor)
                        except serial.SerialException:
                            pass
                    last_status = time.time()
                    last_gripper_status = time.time()
                    last_refresh = time.time()
                    continue

                # Redraw status line every loop tick (catches feedback updates between key presses).
                ts = current_tune(selected_motor)
                fb_key = selected_motor if selected_motor != 0xFF else 1
                fb = last_fb_by_motor.get(fb_key)
                cal = tune_state[fb_key]["_grav_calibrated"]
                print_status(
                    speed_rad_s, direction, args.mode, duty_pct, motor_enabled, selected_motor,
                    ts["kp"], ts["kd"], ts["kg"], bias=ts["bias"],
                    grav_en=ts["grav_en"], last_fb=fb, gripper_fb=last_gripper["frame"],
                    zero_pos_rad=ts["_zero_pos_rad"],
                    grav_calibrated=cal,
                )

        finally:
            # Ensure motor is stopped even if loop exits unexpectedly.
            ensure_stopped()
            try:
                ser.close()
            except Exception:
                pass
            if log_fp is not None:
                log_fp.close()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
