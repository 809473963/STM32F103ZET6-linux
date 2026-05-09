#!/usr/bin/env python3
"""Pi-side TCP bridge for STM32 serial port.

Replaces `socat` for this board because socat does not set DTR=0 / RTS=0,
which leaves NRST held in reset and BOOT0=1 (bootloader mode).

This script:
  1. Opens the serial port with DTR=0  (NRST released, STM32 runs)
                              RTS=0  (BOOT0=0, application mode)
  2. Listens on a TCP port and forwards data bidirectionally.

Usage (on Raspberry Pi):
    python3 pi_serial_tcp_bridge.py
    python3 pi_serial_tcp_bridge.py --serial /dev/ttyUSB0 --baud 115200 --tcp-port 8888

Then on WSL / PC:
    python3 pc_keyboard_control.py --socket-host 192.168.10.2
"""
import argparse
import select
import socket
import time

import serial


def run_bridge(serial_port: str, baud: int, tcp_port: int) -> None:
    # Open serial with correct line states so STM32 stays in application mode.
    ser = serial.Serial()
    ser.port = serial_port
    ser.baudrate = baud
    ser.timeout = 0          # non-blocking read
    ser.dsrdtr = False
    ser.rtscts = False
    ser.open()
    ser.setDTR(False)        # NRST released
    ser.setRTS(False)        # BOOT0 = 0
    time.sleep(0.1)
    ser.flushInput()
    print(f"[bridge] Serial {serial_port} @ {baud}  DTR=0 RTS=0 (app mode)")

    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("0.0.0.0", tcp_port))
    srv.listen(1)
    print(f"[bridge] Listening on TCP :{tcp_port}  (Ctrl-C to stop)")

    ser_fd = ser.fileno()

    while True:
        conn, addr = srv.accept()
        conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        conn_fd = conn.fileno()
        print(f"[bridge] Client connected: {addr}")
        try:
            while True:
                r, _, _ = select.select([ser_fd, conn_fd], [], [], 0.5)
                for fd in r:
                    if fd == ser_fd:
                        data = ser.read(256)
                        if data:
                            conn.sendall(data)
                    elif fd == conn_fd:
                        data = conn.recv(256)
                        if not data:
                            raise ConnectionResetError("client disconnected")
                        ser.write(data)
        except (ConnectionResetError, OSError) as exc:
            print(f"[bridge] Client gone: {exc}")
        finally:
            try:
                conn.close()
            except OSError:
                pass


def main() -> None:
    p = argparse.ArgumentParser(description="Serial-TCP bridge with proper DTR/RTS for STM32")
    p.add_argument("--serial",   default="/dev/ttyUSB0", help="Serial port")
    p.add_argument("--baud",     type=int, default=115200, help="Baud rate")
    p.add_argument("--tcp-port", type=int, default=8888,   help="TCP listen port")
    args = p.parse_args()
    run_bridge(args.serial, args.baud, args.tcp_port)


if __name__ == "__main__":
    main()
