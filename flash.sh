#!/bin/bash
# flash.sh - STM32 Build and Flash utility script
# Uses STM32_Programmer_CLI for flashing

set -e # Exit on error

# Configuration
PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="${PROJECT_ROOT}/build"
FIRMWARE_BIN="${BUILD_DIR}/STM32_KEY_PROJECT.bin"
SERIAL_PORT_DEFAULT="/dev/ttyUSB0"
SERIAL_BAUD_DEFAULT="115200"

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
    echo "  serial [port] [baud] - Flash over UART bootloader (CH340, etc.)"
    echo "  serial-flymcu [port] [baud] - UART flash with FlyMcu-style DTR/RTS timing"
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
        -DCMAKE_BUILD_TYPE=Release
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
    local serial_baud="${2:-$SERIAL_BAUD_DEFAULT}"

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

    if [ ! -e "$serial_port" ]; then
        echo "Error: Serial port $serial_port does not exist."
        echo "Check usbipd attach status and CH340 driver binding in WSL."
        exit 1
    fi

    echo "Tip: Make sure BOOT0=1 and reset the MCU before UART flashing."
    STM32_Programmer_CLI -c port="$serial_port" br="$serial_baud" P=EVEN \
        -w "$FIRMWARE_BIN" 0x08000000 -v -rst

    echo "=== UART Flash Complete ==="
}

flash_firmware_serial_flymcu() {
    local serial_port="${1:-$SERIAL_PORT_DEFAULT}"
    local serial_baud="${2:-$SERIAL_BAUD_DEFAULT}"

    echo "=== Flashing over UART (FlyMcu timing: DTR low reset, RTS high boot) ==="
    if [ ! -f "$FIRMWARE_BIN" ]; then
        echo "Error: Firmware file $FIRMWARE_BIN not found! Run 'build' first."
        exit 1
    fi

    if ! command -v stm32flash &> /dev/null; then
        echo "Error: stm32flash is required for serial-flymcu mode."
        echo "Install it with: sudo apt-get install -y stm32flash"
        exit 1
    fi

    if [ ! -e "$serial_port" ]; then
        echo "Error: Serial port $serial_port does not exist."
        echo "Check usbipd attach status and CH340 driver binding in WSL."
        exit 1
    fi

    # Entry: DTR low + RTS high, then release DTR.
    # Exit: RTS low + DTR high.
    stm32flash -b "$serial_baud" -m 8e1 -w "$FIRMWARE_BIN" -v -R \
        -i '-dtr&rts,dtr:-rts&dtr' "$serial_port"

    echo "=== UART FlyMcu Flash Complete ==="
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
