#!/usr/bin/env python3
"""
Read WHEELTEC chassis feedback from Raspberry Pi serial port and forward it to WSL.

This script is intentionally NOT a ROS node. The Raspberry Pi remains a hardware
adapter only:

  WHEELTEC USB serial -> Raspberry Pi -> UDP -> WSL ROS2

UDP CSV format sent to WSL:
  vx,vy,vz,ax,ay,az,gx,gy,gz,voltage,flag_stop

Units:
  vx/vy: m/s
  vz: rad/s
  ax/ay/az: m/s^2
  gx/gy/gz: rad/s
  voltage: V
"""

import argparse
import socket
import struct
import sys
import time

try:
    import serial
except ImportError:
    print("Missing pyserial. Install with: sudo apt install -y python3-serial")
    sys.exit(1)

FRAME_HEAD = 0x7B
FRAME_TAIL = 0x7D
RECV_LEN = 24


def bcc(data: bytes) -> int:
    value = 0
    for byte in data:
        value ^= byte
    return value


def s16(hi: int, lo: int) -> int:
    value = (hi << 8) | lo
    return value - 65536 if value > 32767 else value


def parse_frame(frame: bytes):
    if len(frame) != RECV_LEN:
        return None
    if frame[0] != FRAME_HEAD or frame[-1] != FRAME_TAIL:
        return None
    if bcc(frame[:22]) != frame[22]:
        return None

    return {
        "flag_stop": frame[1],
        "vx": s16(frame[2], frame[3]) / 1000.0,
        "vy": s16(frame[4], frame[5]) / 1000.0,
        "vz": s16(frame[6], frame[7]) / 1000.0,
        "ax": s16(frame[8], frame[9]) / 1672.0,
        "ay": s16(frame[10], frame[11]) / 1672.0,
        "az": s16(frame[12], frame[13]) / 1672.0,
        "gx": s16(frame[14], frame[15]) / 3755.0,
        "gy": s16(frame[16], frame[17]) / 3755.0,
        "gz": s16(frame[18], frame[19]) / 3755.0,
        "voltage": s16(frame[20], frame[21]) / 1000.0,
    }


def find_frame(buffer: bytearray):
    while len(buffer) >= RECV_LEN:
        try:
            idx = buffer.index(FRAME_HEAD)
        except ValueError:
            buffer.clear()
            return None

        if idx:
            del buffer[:idx]

        if len(buffer) < RECV_LEN:
            return None

        candidate = bytes(buffer[:RECV_LEN])
        parsed = parse_frame(candidate)
        if parsed:
            del buffer[:RECV_LEN]
            return parsed

        del buffer[:1]

    return None


def main():
    parser = argparse.ArgumentParser(description="WHEELTEC serial feedback -> UDP forwarder")
    parser.add_argument("--serial-port", default="/dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--wsl-ip", required=True, help="WSL host IP on the Ethernet link")
    parser.add_argument("--udp-port", type=int, default=15050)
    parser.add_argument("--print", action="store_true", help="Print forwarded state")
    args = parser.parse_args()

    ser = serial.Serial(args.serial_port, args.baud, timeout=0.05, dsrdtr=False, rtscts=False)
    ser.setDTR(False)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    target = (args.wsl_ip, args.udp_port)
    buffer = bytearray()

    print(f"Serial: {args.serial_port} @ {args.baud}")
    print(f"Forwarding chassis state to {target[0]}:{target[1]}")

    try:
        while True:
            chunk = ser.read(128)
            if chunk:
                buffer.extend(chunk)

            state = find_frame(buffer)
            if not state:
                continue

            line = (
                f"{state['vx']:.6f},{state['vy']:.6f},{state['vz']:.6f},"
                f"{state['ax']:.6f},{state['ay']:.6f},{state['az']:.6f},"
                f"{state['gx']:.6f},{state['gy']:.6f},{state['gz']:.6f},"
                f"{state['voltage']:.3f},{state['flag_stop']}"
            )
            sock.sendto(line.encode("ascii"), target)

            if args.print:
                print(line)

    except KeyboardInterrupt:
        pass
    finally:
        ser.close()
        sock.close()


if __name__ == "__main__":
    main()
