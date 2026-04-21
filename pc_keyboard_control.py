#!/usr/bin/env python3
"""PC keyboard teleop for STM32 motor controller over serial.

Protocol frame:
  [0xAA] [LEN] [CMD] [DATA...] [CHECKSUM]
  CHECKSUM = (LEN + CMD + DATA bytes) & 0xFF
"""

import argparse
import glob
import os
import select
import struct
import subprocess
import sys
import termios
import tty
import time

import serial

HEADER = 0xAA
CMD_SET_MOTOR_SPEED = 0x01
CMD_STOP_ALL = 0x02
CMD_SET_ENABLE = 0x04
CMD_SET_MOTOR_VEL_MRAD = 0x05

DIR_FORWARD = 1
DIR_BACKWARD = 0
DIR_STOP = 2

# Must match firmware side mapping in protocol.c.
VEL_MAX_RAD_S = 14.0


class RawTerminal:
    def __init__(self):
        self._fd = sys.stdin.fileno()
        self._old = termios.tcgetattr(self._fd)

    def __enter__(self):
        tty.setraw(self._fd)
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


def force_stop_motor(ser, motor_id):
    send_stop_all(ser)
    send_enable(ser, motor_id, 0)
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


def poll_device_messages(ser, budget_s=0.01):
    end_t = time.time() + budget_s
    while time.time() < end_t:
        waiting = getattr(ser, "in_waiting", 0)
        if waiting <= 0:
            break
        raw = ser.readline()
        if not raw:
            break
        msg = raw.decode("ascii", errors="replace").strip()
        if msg:
            print(f"\n{msg}")


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
                return serial.Serial(active_port, baud, timeout=0.1), active_port
            except Exception as exc:
                last_error = str(exc)
                if "Permission denied" in last_error:
                    try_fix_permission(active_port)
                    try:
                        return serial.Serial(active_port, baud, timeout=0.1), active_port
                    except Exception as exc2:
                        last_error = str(exc2)

        if recover_timeout <= 0:
            raise RuntimeError(last_error)
        if time.time() - start > recover_timeout:
            raise RuntimeError(last_error)
        time.sleep(0.6)


def print_status(speed_rad_s, direction, mode=None, duty_pct=None, enabled=True):
    direction_text = "FORWARD" if direction == DIR_FORWARD else "BACKWARD"
    enabled_text = "EN" if enabled else "DIS"
    if mode == "duty":
        show_duty = 0 if duty_pct is None else int(round(duty_pct))
        print(
            f"\rDuty: {show_duty:3d}%  Direction: {direction_text:8s}  Motor:{enabled_text}  ",
            end="",
            flush=True,
        )
    else:
        print(
            f"\rSpeed: {speed_rad_s:4.2f} rad/s  Direction: {direction_text:8s}  Motor:{enabled_text}  ",
            end="",
            flush=True,
        )


def main():
    parser = argparse.ArgumentParser(description="Keyboard control STM32 motor via serial")
    parser.add_argument("--port", default="/dev/ttyUSB0", help="Serial port, e.g. /dev/ttyUSB0")
    parser.add_argument("--socket-host", default=None, help="Use TCP serial bridge host, e.g. 192.168.10.2")
    parser.add_argument("--socket-port", type=int, default=8888, help="TCP serial bridge port")
    parser.add_argument("--baud", type=int, default=115200, help="Baudrate")
    parser.add_argument("--motor", type=int, default=1, choices=[1, 2, 3, 4], help="Motor id (1-4)")
    parser.add_argument("--mode", choices=["duty", "vel"], default="duty", help="Control mode for A/D")
    parser.add_argument("--step", type=int, default=35, help="Hold duty percent for A/D in duty mode")
    parser.add_argument("--speed-rad-s", type=float, default=None, help="Hold speed for A/D in rad/s (overrides --step)")
    parser.add_argument("--recover-timeout", type=int, default=30, help="Seconds to wait/recover serial after replug (0 to disable)")
    parser.add_argument("--refresh-ms", type=int, default=120, help="Resend current command periodically (ms)")
    parser.add_argument("--hold-timeout-ms", type=int, default=0, help="Release timeout for A/D hold (ms), 0 to disable")
    args = parser.parse_args()

    if args.socket_host:
        args.port = f"socket://{args.socket_host}:{args.socket_port}"

    speed_rad_s = 0.0
    duty_pct = 0.0
    direction = DIR_FORWARD
    if args.speed_rad_s is not None:
        hold_speed_rad_s = max(0.05, min(VEL_MAX_RAD_S, args.speed_rad_s))
    else:
        hold_speed_pct = max(1, min(100, args.step))
        hold_speed_rad_s = (hold_speed_pct * VEL_MAX_RAD_S) / 100.0

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
    if args.mode == "duty":
        print("Controls: A backward, D forward, W/S adjust duty, Space stop, Q quit")
        print(f"Hold duty: {hold_duty_pct:.0f}%")
    else:
        print("Controls: A backward, D forward, W/S adjust speed, Space stop, Q quit")
        print(f"Hold speed: {actual_hold_rad_s:.2f} rad/s")

    try:
        ser, real_port = open_serial_with_recover(args.port, args.baud, args.recover_timeout)
        ser.dtr = False
        ser.rts = False
        print(f"Using serial port: {real_port}")
    except Exception as exc:
        print(f"Open serial failed: {exc}")
        print("Try: sudo chmod 666 /dev/ttyUSB0")
        return 1

    with ser, RawTerminal():
        send_enable(ser, args.motor, 1)
        print_status(speed_rad_s, direction, args.mode, duty_pct, motor_enabled)
        last_refresh = time.time()
        while True:
            rlist, _, _ = select.select([sys.stdin], [], [], 0.05)
            ch = sys.stdin.read(1) if rlist else ""

            if ch:
                c = ch.lower()
                if c == "q":
                    speed_rad_s = 0.0
                    duty_pct = 0.0
                    motor_enabled = False
                    force_stop_motor(ser, args.motor)
                    print("\nQuit and stop all motors.")
                    break

                if c == "a":
                    if direction != DIR_BACKWARD:
                        # Direction switch is more stable if pulse is dropped briefly first.
                        if args.mode == "duty":
                            send_set_motor(ser, args.motor, 0, DIR_STOP)
                        else:
                            send_set_motor_vel_mrad(ser, args.motor, 0.0)
                        time.sleep(0.05)
                    direction = DIR_BACKWARD
                    speed_rad_s = hold_speed_rad_s
                    duty_pct = hold_duty_pct
                    drive_active = True
                    motor_enabled = True
                    last_drive_key_time = time.time()
                    if args.mode == "duty":
                        apply_drive_state_duty(ser, args.motor, duty_pct, direction)
                    else:
                        apply_drive_state(ser, args.motor, speed_rad_s, direction)
                elif c == "d":
                    if direction != DIR_FORWARD:
                        if args.mode == "duty":
                            send_set_motor(ser, args.motor, 0, DIR_STOP)
                        else:
                            send_set_motor_vel_mrad(ser, args.motor, 0.0)
                        time.sleep(0.05)
                    direction = DIR_FORWARD
                    speed_rad_s = hold_speed_rad_s
                    duty_pct = hold_duty_pct
                    drive_active = True
                    motor_enabled = True
                    last_drive_key_time = time.time()
                    if args.mode == "duty":
                        apply_drive_state_duty(ser, args.motor, duty_pct, direction)
                    else:
                        apply_drive_state(ser, args.motor, speed_rad_s, direction)
                elif c == " ":
                    speed_rad_s = 0.0
                    duty_pct = 0.0
                    drive_active = False
                    motor_enabled = False
                    if args.mode == "duty":
                        apply_drive_state_duty(ser, args.motor, duty_pct, direction)
                    else:
                        apply_drive_state(ser, args.motor, speed_rad_s, direction)
                elif c == "w":
                    if args.mode == "duty":
                        hold_duty_pct = min(100.0, hold_duty_pct + 2.0)
                        if drive_active:
                            duty_pct = hold_duty_pct
                            apply_drive_state_duty(ser, args.motor, duty_pct, direction)
                    else:
                        hold_speed_rad_s = min(VEL_MAX_RAD_S, hold_speed_rad_s + 0.1)
                        if drive_active:
                            speed_rad_s = hold_speed_rad_s
                            apply_drive_state(ser, args.motor, speed_rad_s, direction)
                elif c == "s":
                    if args.mode == "duty":
                        hold_duty_pct = max(1.0, hold_duty_pct - 2.0)
                        if drive_active:
                            duty_pct = hold_duty_pct
                            apply_drive_state_duty(ser, args.motor, duty_pct, direction)
                    else:
                        hold_speed_rad_s = max(0.05, hold_speed_rad_s - 0.1)
                        if drive_active:
                            speed_rad_s = hold_speed_rad_s
                            apply_drive_state(ser, args.motor, speed_rad_s, direction)

                last_refresh = time.time()
                print_status(speed_rad_s, direction, args.mode, duty_pct, motor_enabled)

            if hold_timeout_s is not None and drive_active and (time.time() - last_drive_key_time) > hold_timeout_s:
                drive_active = False
                if speed_rad_s != 0.0 or duty_pct != 0.0:
                    speed_rad_s = 0.0
                    duty_pct = 0.0
                    motor_enabled = False
                    if args.mode == "duty":
                        apply_drive_state_duty(ser, args.motor, duty_pct, direction)
                    else:
                        apply_drive_state(ser, args.motor, speed_rad_s, direction)
                    print_status(speed_rad_s, direction, args.mode, duty_pct, motor_enabled)

            if drive_active and speed_rad_s > 0.0 and (time.time() - last_refresh) >= refresh_s:
                motor_enabled = True
                if args.mode == "duty":
                    apply_drive_state_duty(ser, args.motor, duty_pct, direction)
                else:
                    apply_drive_state(ser, args.motor, speed_rad_s, direction)
                last_refresh = time.time()

            poll_device_messages(ser)

        # Ensure motor is stopped even if loop exits unexpectedly.
        force_stop_motor(ser, args.motor)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
