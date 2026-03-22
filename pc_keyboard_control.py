#!/usr/bin/env python3
"""PC keyboard teleop for STM32 motor controller over serial.

Protocol frame:
  [0xAA] [LEN] [CMD] [DATA...] [CHECKSUM]
  CHECKSUM = (LEN + CMD + DATA bytes) & 0xFF
"""

import argparse
import glob
import os
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

DIR_FORWARD = 1
DIR_BACKWARD = 0
DIR_STOP = 2


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
    send_enable(ser, motor_id, 1)
    frame = make_frame(CMD_SET_MOTOR_SPEED, [motor_id, speed_u8, direction])
    ser.write(frame)


def send_stop_all(ser):
    frame = make_frame(CMD_STOP_ALL, [])
    ser.write(frame)


def send_enable(ser, motor_id, enable):
    frame = make_frame(CMD_SET_ENABLE, [motor_id, 1 if enable else 0])
    ser.write(frame)


def detect_ttyusb_port(preferred=None):
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


def print_status(speed, direction):
    direction_text = "FORWARD" if direction == DIR_FORWARD else "BACKWARD"
    print(f"\rSpeed: {speed:3d}%  Direction: {direction_text:8s}  ", end="", flush=True)


def main():
    parser = argparse.ArgumentParser(description="Keyboard control STM32 motor via serial")
    parser.add_argument("--port", default="/dev/ttyUSB0", help="Serial port, e.g. /dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=115200, help="Baudrate")
    parser.add_argument("--motor", type=int, default=1, choices=[1, 2, 3, 4], help="Motor id (1-4)")
    parser.add_argument("--step", type=int, default=10, help="Speed step percent")
    parser.add_argument("--recover-timeout", type=int, default=30, help="Seconds to wait/recover serial after replug (0 to disable)")
    args = parser.parse_args()

    speed = 0
    direction = DIR_FORWARD
    step = max(1, min(50, args.step))

    print("Keyboard control started")
    print("Controls: W speed+, S speed-, A backward, D forward, Space stop, Q quit")

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
        print_status(speed, direction)
        while True:
            ch = sys.stdin.read(1)
            if not ch:
                continue

            c = ch.lower()
            if c == "q":
                send_stop_all(ser)
                print("\nQuit and stop all motors.")
                break
            if c == "w":
                speed = min(100, speed + step)
                if speed == 0:
                    send_stop_all(ser)
                else:
                    send_set_motor(ser, args.motor, speed, direction)
            elif c == "s":
                speed = max(0, speed - step)
                if speed == 0:
                    send_stop_all(ser)
                else:
                    send_set_motor(ser, args.motor, speed, direction)
            elif c == "a":
                direction = DIR_BACKWARD
                if speed == 0:
                    send_stop_all(ser)
                else:
                    send_set_motor(ser, args.motor, speed, direction)
            elif c == "d":
                direction = DIR_FORWARD
                if speed == 0:
                    send_stop_all(ser)
                else:
                    send_set_motor(ser, args.motor, speed, direction)
            elif c == " ":
                speed = 0
                send_stop_all(ser)

            print_status(speed, direction)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
