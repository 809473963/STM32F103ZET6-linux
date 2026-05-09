#!/usr/bin/env bash
set -euo pipefail

HOST="192.168.10.2"
USER_NAME="ubuntu"
SERIAL_PORT="/dev/ttyUSB0"
BAUD="115200"
FW_PATH=""

usage() {
    echo "Usage: $0 --fw <firmware.bin> [--user ubuntu] [--host 192.168.10.2] [--serial /dev/ttyUSB0] [--baud 115200]"
    echo "Example: $0 --fw /home/luo/stm-linux/build/STM32_KEY_PROJECT.bin --user ubuntu"
    echo "Example: $0 --fw STM32_KEY_PROJECT.bin --user ubuntu  (auto-search in build directory)"
}

resolve_fw_path() {
    local input_path="$1"
    local candidate=""
    local best_path=""
    local best_mtime=0
    local mtime=0
    local hit_count=0
    local base_name

    if [[ -f "$input_path" ]]; then
        echo "$input_path"
        return 0
    fi

    base_name="$(basename "$input_path")"
    for candidate in \
        "./$base_name" \
        "./build/$base_name"; do
        if [[ -f "$candidate" ]]; then
            hit_count=$((hit_count + 1))
            mtime=$(stat -c %Y "$candidate" 2>/dev/null || echo 0)
            if (( mtime >= best_mtime )); then
                best_mtime=$mtime
                best_path="$candidate"
            fi
        fi
    done

    if (( hit_count == 0 )); then
        return 1
    fi

    if (( hit_count > 1 )); then
        echo "Warning: multiple firmware matches found for '$base_name', using latest: $best_path" >&2
    fi

    echo "$best_path"
    return 0
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --fw)
            FW_PATH="$2"
            shift 2
            ;;
        --user)
            USER_NAME="$2"
            shift 2
            ;;
        --host)
            HOST="$2"
            shift 2
            ;;
        --serial)
            SERIAL_PORT="$2"
            shift 2
            ;;
        --baud)
            BAUD="$2"
            shift 2
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            echo "Unknown argument: $1"
            usage
            exit 1
            ;;
    esac
done

if [[ -z "$FW_PATH" ]]; then
    echo "Error: --fw is required"
    usage
    exit 1
fi

if ! FW_PATH=$(resolve_fw_path "$FW_PATH"); then
    echo "Error: firmware not found: $FW_PATH"
    echo "Hint: available candidates are:"
    ls -1 ./build*/STM32_KEY_PROJECT.bin 2>/dev/null || true
    exit 1
fi

echo "Using firmware: $FW_PATH"

REMOTE_FW="/tmp/stm32_fw_$(date +%Y%m%d_%H%M%S).bin"
SSH_TARGET="${USER_NAME}@${HOST}"

echo "[1/3] Upload firmware to Raspberry Pi: $SSH_TARGET:$REMOTE_FW"
scp -o StrictHostKeyChecking=accept-new "$FW_PATH" "$SSH_TARGET:$REMOTE_FW"

echo "[2/3] Run flashing on Raspberry Pi"
ssh "$SSH_TARGET" "bash -s" <<EOF
set -euo pipefail

SERIAL_PORT="$SERIAL_PORT"
BAUD="$BAUD"
REMOTE_FW="$REMOTE_FW"

if ! command -v stm32flash >/dev/null 2>&1; then
    echo "stm32flash is missing on Raspberry Pi, installing..."
    sudo apt-get update
    sudo apt-get install -y stm32flash
fi

if [[ ! -e "\$SERIAL_PORT" ]]; then
    echo "Error: serial port not found on Raspberry Pi: \$SERIAL_PORT"
    ls -l /dev/ttyUSB* /dev/ttyACM* 2>/dev/null || true
    exit 1
fi

echo "Preparing serial port before flashing: \$SERIAL_PORT"
# Kill common forwarders/users that may hold the serial fd (e.g. socat bridge).
sudo pkill -f "socat.*\$SERIAL_PORT" 2>/dev/null || true
sudo fuser -k "\$SERIAL_PORT" 2>/dev/null || true
sleep 0.2

# Best-effort line discipline reset to reduce stale tty state impact.
stty -F "\$SERIAL_PORT" 115200 cs8 -cstopb -parenb -ixon -ixoff -crtscts raw -echo || true
sleep 0.1

flash_once() {
    local baud="\$1"
    local mode="\$2"

    if [[ "\$mode" == "flymcu" ]]; then
        echo "Attempt flymcu timing, baud=\$baud"
        sudo stm32flash -b "\$baud" -m 8e1 -w "\$REMOTE_FW" -v -R -i '-dtr&rts,dtr:-rts&dtr' "\$SERIAL_PORT"
    else
        echo "Attempt manual boot mode, baud=\$baud"
        sudo stm32flash -b "\$baud" -m 8e1 -w "\$REMOTE_FW" -v -R "\$SERIAL_PORT"
    fi
}

if flash_once "\$BAUD" flymcu || flash_once 57600 flymcu || flash_once 38400 flymcu || \
   flash_once "\$BAUD" flymcu || flash_once 57600 flymcu; then
    echo "Flash success (flymcu path, with retries)."
else
    echo ""
    echo "=========================================="
    echo " Flymcu auto-reset failed."
    echo " ACTION REQUIRED:"
    echo "   1. Set BOOT0 = 1 on the STM32 board"
    echo "   2. Power-cycle (unplug and replug) STM32"
    echo "   3. Press ENTER here to start flashing"
    echo "=========================================="
    read -r _
    if flash_once "\$BAUD" manual || flash_once 57600 manual || flash_once 38400 manual || \
       flash_once "\$BAUD" manual; then
        echo "Flash success (manual BOOT0 path, with retries)."
    else
        echo "Flash failed on Raspberry Pi."
        exit 1
    fi
fi

# ---------------------------------------------------------------
# Post-flash cleanup: ensure RTS=0 (BOOT0=0) and trigger one
# clean reset so the STM32 starts in application mode.
# Without this, the port may be left with RTS=1 after stm32flash
# closes it (OS default), and the next open will see BOOT0=1.
# ---------------------------------------------------------------
python3 - <<PYEOF
import serial, time, sys
PORT = "\$SERIAL_PORT"
BAUD = \$BAUD
try:
    s = serial.Serial()
    s.port = PORT
    s.baudrate = BAUD
    s.dsrdtr  = False   # prevent pyserial from touching DTR on open
    s.rtscts  = False
    s.open()
    s.setRTS(False)     # BOOT0 = 0  (application mode)
    time.sleep(0.05)
    s.setDTR(True)      # assert NRST (begin reset pulse)
    time.sleep(0.05)
    s.setDTR(False)     # release NRST → STM32 boots with BOOT0=0
    time.sleep(0.05)
    s.close()
    print("[post-flash] RTS=0 DTR=0  → STM32 reset into application mode.")
except Exception as e:
    print(f"[post-flash] WARNING: could not clear RTS/DTR: {e}", file=sys.stderr)
PYEOF

rm -f "\$REMOTE_FW"
EOF

echo "[3/3] Done"
