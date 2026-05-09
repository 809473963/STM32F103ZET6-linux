#!/bin/bash
# flash.sh - STM32 Build and Flash utility script
# Uses STM32_Programmer_CLI for flashing

set -e # Exit on error

# Configuration
PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="${PROJECT_ROOT}/build"
FIRMWARE_BIN="${BUILD_DIR}/STM32_KEY_PROJECT.bin"
SERIAL_PORT_DEFAULT="auto"
SERIAL_BAUD_DEFAULT="115200"

detect_serial_port() {
    local requested_port="$1"
    local candidate
    local candidates=()

    if [ -n "$requested_port" ] && [ -e "$requested_port" ]; then
        echo "$requested_port"
        return 0
    fi

    while IFS= read -r candidate; do
        candidates+=("$candidate")
    done < <(ls -1 /dev/ttyUSB* /dev/ttyACM* 2>/dev/null | sort -V)

    if [ "${#candidates[@]}" -eq 0 ]; then
        return 1
    fi

    # Prefer the most recently numbered node (common after replug in WSL/USBIP).
    echo "${candidates[-1]}"
    return 0
}

ensure_build_dir() {
    local cache_file="${BUILD_DIR}/CMakeCache.txt"
    local cached_source=""

    if [ -f "$cache_file" ]; then
        cached_source=$(grep '^CMAKE_HOME_DIRECTORY:INTERNAL=' "$cache_file" | cut -d= -f2- || true)
        if [ -n "$cached_source" ] && [ "$cached_source" != "$PROJECT_ROOT" ]; then
            echo "Detected stale CMake cache (source: $cached_source). Recreating build directory..."
            rm -rf "$BUILD_DIR"
        fi
    fi

    mkdir -p "$BUILD_DIR"
}

# Help message
print_usage() {
    echo "Usage: ./flash.sh [command]"
    echo ""
    echo "Commands:"
    echo "  (empty)   - Build and Flash directly"
    echo "  build     - Only build the firmware"
    echo "  flash     - Only flash the existing firmware"
    echo "  arm-dm-build - Build ARM DM firmware (alias of build)"
    echo "  arm-dm-serial-flymcu [port] [baud] - Flash ARM DM app over UART (alias of serial-flymcu)"
    echo "  serial [port] [baud] - Flash over UART bootloader (CH340, etc.; port optional, auto-detect if missing)"
    echo "  serial-flymcu [port] [baud] - UART flash with FlyMcu-style DTR/RTS timing (port optional, auto-detect if missing)"
    echo "  clean     - Clean the build directory"
    echo "  help      - Show this message"
}

build_firmware() {
    echo "=== Building STM32 firmware ==="
    ensure_build_dir
    cmake \
        -S "$PROJECT_ROOT" \
        -B "$BUILD_DIR" \
        -DCMAKE_TOOLCHAIN_FILE="$PROJECT_ROOT/arm-none-eabi-gcc.cmake" \
        -DCMAKE_BUILD_TYPE=Release \
        -DARM_DM_APP=ON
    cmake --build "$BUILD_DIR" -j"$(nproc)"
    echo "=== Build Complete ==="
}

flash_firmware() {
    echo "=== Flashing to STM32 ==="
    if [ ! -f "$FIRMWARE_BIN" ]; then
        echo "Error: Firmware file $FIRMWARE_BIN not found! Run 'build' first."
        exit 1
    fi
    
    # Check if STM32_Programmer_CLI is in PATH
    if ! command -v STM32_Programmer_CLI &> /dev/null; then
        echo "Error: STM32_Programmer_CLI could not be found."
        echo "Please make sure STM32CubeProgrammer is installed and added to your PATH."
        echo "Default path: /usr/local/STMicroelectronics/STM32Cube/STM32CubeProgrammer/bin"
        exit 1
    fi
    
    # Try USB first (if directly connected as mass storage/DFU), fallback to SWD (ST-Link)
    echo "Attempting to flash via SWD (ST-Link)..."
    STM32_Programmer_CLI -c port=SWD -w "$FIRMWARE_BIN" 0x08000000 -v -rst || \
    { echo "SWD failed, trying direct USB DFU..."; STM32_Programmer_CLI -c port=USB1 -w "$FIRMWARE_BIN" 0x08000000 -v -rst; }
    
    echo "=== Flash Complete ==="
}

flash_firmware_serial() {
    local serial_port="${1:-$SERIAL_PORT_DEFAULT}"
    local original_port="$serial_port"
    local serial_baud="${2:-$SERIAL_BAUD_DEFAULT}"

    if ! serial_port=$(detect_serial_port "$serial_port"); then
        echo "Error: Serial port $original_port does not exist, and no /dev/ttyUSB* or /dev/ttyACM* was detected."
        echo "Check USB passthrough and run: ls -l /dev/ttyUSB* /dev/ttyACM*"
        exit 1
    fi

    if [ "$serial_port" != "$original_port" ]; then
        echo "Auto-detected serial port: $serial_port (requested: $original_port)"
    fi

    echo "=== Flashing to STM32 over UART (${serial_port}, ${serial_baud}) ==="
    if [ ! -f "$FIRMWARE_BIN" ]; then
        echo "Error: Firmware file $FIRMWARE_BIN not found! Run 'build' first."
        exit 1
    fi

    if ! command -v STM32_Programmer_CLI &> /dev/null; then
        echo "Error: STM32_Programmer_CLI could not be found."
        echo "Please make sure STM32CubeProgrammer is installed and added to your PATH."
        exit 1
    fi

    echo "Tip: Make sure BOOT0=1 and reset the MCU before UART flashing."
    STM32_Programmer_CLI -c port="$serial_port" br="$serial_baud" P=EVEN \
        -w "$FIRMWARE_BIN" 0x08000000 -v -rst

    echo "=== UART Flash Complete ==="
}

run_stm32flash_write() {
    local serial_port="$1"
    local serial_baud="$2"
    local firmware_bin="$3"
    local use_flymcu_timing="$4"

    local -a cmd=(stm32flash -b "$serial_baud" -m 8e1 -w "$firmware_bin" -v -R)
    if [ "$use_flymcu_timing" = "1" ]; then
        # Entry: DTR low + RTS high, then release DTR.
        # Exit: RTS low + DTR high.
        cmd+=(-i '-dtr&rts,dtr:-rts&dtr')
    fi
    cmd+=("$serial_port")

    if [ -w "$serial_port" ]; then
        "${cmd[@]}"
    else
        echo "No write permission on $serial_port, retrying with sudo..."
        sudo "${cmd[@]}"
    fi
}

flash_with_stm32flash_retries() {
    local firmware_bin="$1"
    local serial_port="$2"
    local serial_baud="$3"
    local title="$4"

    local -a bauds=("$serial_baud" 57600 38400)
    local tried=""
    local b=""

    echo "=== $title ==="

    # Phase 1: try FlyMcu DTR/RTS timing first.
    for b in "${bauds[@]}"; do
        if [[ " $tried " == *" $b "* ]]; then
            continue
        fi
        tried+=" $b"
        echo "Attempt (FlyMcu timing): baud=$b"
        if run_stm32flash_write "$serial_port" "$b" "$firmware_bin" 1; then
            echo "=== UART FlyMcu Flash Complete ==="
            return 0
        fi
    done

    echo "FlyMcu timing failed for all baud rates."
    echo "Fallback: Please set BOOT0=1 manually, press RESET once, then keep board in system bootloader."

    # Phase 2: manual BOOT0 fallback without DTR/RTS sequence.
    for b in "${bauds[@]}"; do
        echo "Attempt (manual BOOT0 mode): baud=$b"
        if run_stm32flash_write "$serial_port" "$b" "$firmware_bin" 0; then
            echo "=== UART Manual-Boot Flash Complete ==="
            return 0
        fi
    done

    echo "Error: Unable to enter STM32 serial bootloader."
    echo "Checklist:"
    echo "  1) BOOT0=1 before reset, BOOT0=0 after flash"
    echo "  2) TX/RX/GND wiring is correct"
    echo "  3) Serial port is not occupied (fuser -v $serial_port)"
    return 1
}

flash_firmware_serial_flymcu() {
    local serial_port="${1:-$SERIAL_PORT_DEFAULT}"
    local original_port="$serial_port"
    local serial_baud="${2:-$SERIAL_BAUD_DEFAULT}"

    if ! serial_port=$(detect_serial_port "$serial_port"); then
        echo "Error: Serial port $original_port does not exist, and no /dev/ttyUSB* or /dev/ttyACM* was detected."
        echo "Check USB passthrough and run: ls -l /dev/ttyUSB* /dev/ttyACM*"
        exit 1
    fi

    if [ "$serial_port" != "$original_port" ]; then
        echo "Auto-detected serial port: $serial_port (requested: $original_port)"
    fi

    if [ ! -f "$FIRMWARE_BIN" ]; then
        echo "Error: Firmware file $FIRMWARE_BIN not found! Run 'build' first."
        exit 1
    fi

    if ! command -v stm32flash &> /dev/null; then
        echo "Error: stm32flash is required for serial-flymcu mode."
        echo "Install it with: sudo apt-get install -y stm32flash"
        exit 1
    fi

    flash_with_stm32flash_retries "$FIRMWARE_BIN" "$serial_port" "$serial_baud" \
        "Flashing over UART (FlyMcu timing + manual BOOT0 fallback)"
}

clean_build() {
    echo "=== Cleaning build directory ==="
    rm -rf "$BUILD_DIR"
    echo "=== Clean Complete ==="
}

# Parse commands
case "$1" in
    "build")
        build_firmware
        ;;
    "flash")
        flash_firmware
        ;;
    "arm-dm-build")
        build_firmware
        ;;
    "arm-dm-serial-flymcu")
        flash_firmware_serial_flymcu "$2" "$3"
        ;;
    "serial")
        flash_firmware_serial "$2" "$3"
        ;;
    "serial-flymcu")
        flash_firmware_serial_flymcu "$2" "$3"
        ;;
    "clean")
        clean_build
        ;;
    "help"|"-h"|"--help")
        print_usage
        ;;
    "")
        build_firmware
        flash_firmware
        ;;
    *)
        echo "Unknown command: $1"
        print_usage
        exit 1
        ;;
esac

exit 0
