import argparse
import struct
import time

import serial

HEADER = 0xAA
CMD_STOP_ALL = 0x02
CMD_GET_STATUS = 0x03
CMD_SET_ENABLE = 0x04
CMD_SET_MOTOR_VEL_MRAD = 0x05
CMD_SET_MOTOR_SPEED = 0x01
CMD_STATUS = 0x83

STATUS_PAYLOAD_STRUCT = struct.Struct("<BBBBfffB")


def make_frame(cmd: int, payload: bytes) -> bytes:
    length = len(payload)
    checksum = (length + cmd + sum(payload)) & 0xFF
    return bytes([HEADER, length, cmd]) + payload + bytes([checksum])


def try_parse_frames(rx_buf: bytearray) -> None:
    while True:
        if len(rx_buf) < 4:
            return

        if rx_buf[0] != HEADER:
            del rx_buf[0]
            continue

        frame_len = rx_buf[1]
        total = frame_len + 4
        if len(rx_buf) < total:
            return

        frame = bytes(rx_buf[:total])
        del rx_buf[:total]

        cmd = frame[2]
        payload = frame[3:-1]
        got_cs = frame[-1]
        calc_cs = (frame[1] + frame[2] + sum(payload)) & 0xFF
        if got_cs != calc_cs:
            print(f"[STM32 -> WSL] bad checksum: got=0x{got_cs:02X}, calc=0x{calc_cs:02X}")
            continue

        if cmd == CMD_STATUS:
            if payload == b"OK":
                print("[STM32 -> WSL] status legacy payload: OK")
                continue

            if len(payload) == STATUS_PAYLOAD_STRUCT.size:
                motor_group, control_mode, motor_id, enabled, pos, vel, effort, status = STATUS_PAYLOAD_STRUCT.unpack(
                    payload
                )
                print(
                    "[STM32 -> WSL] status"
                    f" group={motor_group} mode={control_mode} id={motor_id}"
                    f" enabled={enabled} pos={pos:.3f}rad vel={vel:.3f}rad/s"
                    f" effort={effort:.3f}Nm status=0x{status:02X}"
                )
                continue

        payload_hex = " ".join(f"{b:02X}" for b in payload)
        print(f"[STM32 -> WSL] frame cmd=0x{cmd:02X}, payload=[{payload_hex}]")


def main() -> None:
    parser = argparse.ArgumentParser(description="WSL <-> STM32 bridge test (hello or protocol)")
    parser.add_argument("--serial-port", default=None, help="Direct serial port, e.g. /dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=115200, help="Baudrate for direct serial mode")
    parser.add_argument("--host", default="192.168.10.2", help="Raspberry Pi IP")
    parser.add_argument("--port", type=int, default=8888, help="socat TCP port")
    parser.add_argument(
        "--mode",
        choices=["hello", "status", "enable", "vel", "speed", "stop"],
        default="status",
        help="Communication mode",
    )
    parser.add_argument("--motor", type=int, default=1, help="Motor id for enable/vel")
    parser.add_argument("--enable", type=int, choices=[0, 1], default=1, help="Enable flag for mode=enable")
    parser.add_argument("--vel-mrad", type=int, default=1200, help="Velocity command (mrad/s) for mode=vel")
    parser.add_argument("--speed", type=int, default=35, help="Duty percent for mode=speed (0-100)")
    parser.add_argument("--dir", type=int, choices=[0, 1, 2], default=1, help="Direction for mode=speed: 1=fwd 0=back 2=stop")
    parser.add_argument("--once", action="store_true", help="Send once then exit")
    parser.add_argument("--interval", type=float, default=1.0, help="seconds between hello")
    parser.add_argument("--timeout", type=float, default=1.0, help="socket read timeout")
    args = parser.parse_args()

    url = f"socket://{args.host}:{args.port}"
    ser = None
    try:
        if args.serial_port:
            print(f"Connecting to serial {args.serial_port} @ {args.baud} ...")
            ser = serial.Serial(args.serial_port, args.baud, timeout=args.timeout)
        else:
            print(f"Connecting to {url} ...")
            ser = serial.serial_for_url(url, timeout=args.timeout)
        print(f"Connected. mode={args.mode}. Press Ctrl+C to stop.")

        rx_buf = bytearray()

        while True:
            if args.mode == "hello":
                ser.write(b"hello\n")
                ser.flush()
                print("[WSL -> STM32] hello")

                recv = ser.readline().decode("utf-8", errors="ignore").strip()
                if recv:
                    print(f"[STM32 -> WSL] {recv}")
                    if recv == "ack":
                        print("Handshake OK")
                    else:
                        print("Unexpected response, expected: ack")
                else:
                    print("No response (timeout)")
            else:
                if args.mode == "status":
                    frame = make_frame(CMD_GET_STATUS, b"")
                elif args.mode == "enable":
                    frame = make_frame(CMD_SET_ENABLE, bytes([args.motor & 0xFF, args.enable & 0xFF]))
                elif args.mode == "vel":
                    vel_i16 = max(-32768, min(32767, int(args.vel_mrad)))
                    vel_bytes = struct.pack("<h", vel_i16)
                    frame = make_frame(CMD_SET_MOTOR_VEL_MRAD, bytes([args.motor & 0xFF, vel_bytes[0], vel_bytes[1]]))
                elif args.mode == "speed":
                    speed_u8 = max(0, min(100, int(args.speed)))
                    frame = make_frame(CMD_SET_MOTOR_SPEED, bytes([args.motor & 0xFF, speed_u8, args.dir & 0xFF]))
                else:  # stop
                    frame = make_frame(CMD_STOP_ALL, b"")

                ser.write(frame)
                ser.flush()
                print("[WSL -> STM32]", " ".join(f"{b:02X}" for b in frame))

                time.sleep(0.05)
                waiting = getattr(ser, "in_waiting", 0)
                if waiting > 0:
                    rx_buf.extend(ser.read(waiting))
                    try_parse_frames(rx_buf)
                else:
                    print("[STM32 -> WSL] no frame yet")

            if args.once:
                break

            time.sleep(args.interval)

    except ConnectionRefusedError:
        print("Connection refused. Ensure socat is running on Raspberry Pi.")
    except KeyboardInterrupt:
        print("Stopped by user.")
    except Exception as exc:
        print(f"Error: {exc}")
    finally:
        if ser is not None and ser.is_open:
            ser.close()


if __name__ == "__main__":
    main()