# stm-linux

Linux-based STM32F103ZET6 firmware build and flash workflow using GCC Arm Embedded, CMake, and STM32CubeProgrammer.

## Overview

This project ports a Keil-style STM32F103ZE firmware layout to a Linux-friendly toolchain.

- MCU: STM32F103ZE (Cortex-M3)
- Toolchain: `arm-none-eabi-gcc`
- Build system: CMake
- Programmer: STM32CubeProgrammer CLI (`STM32_Programmer_CLI`)
- Outputs: `.elf`, `.hex`, `.bin`

## Directory Layout

- `CORE/`, `HARDWARE/`, `SYSTEM/`, `USER/`: firmware source tree
- `STM32F10x_FWLib/`: STM32 Standard Peripheral Library
- `CMakeLists.txt`: project build entry
- `arm-none-eabi-gcc.cmake`: GCC cross-compile toolchain file
- `STM32F103ZETx_FLASH.ld`: linker script
- `flash.sh`: build/flash helper
- `setup.sh`: development environment setup helper
- `install_cubeprogrammer.sh`: CubeProgrammer installer helper

## Prerequisites

On Ubuntu/Debian:

```bash
sudo apt-get update
sudo apt-get install -y gcc-arm-none-eabi binutils-arm-none-eabi gdb-multiarch \
  cmake ninja-build make unzip libnewlib-arm-none-eabi
```

Install STM32CubeProgrammer and ensure CLI is in `PATH`:

```bash
STM32_Programmer_CLI --version
```

If command is not found, run:

```bash
./install_cubeprogrammer.sh
source ~/.bashrc
```

## Quick Start

1. Set executable bits (first time only):

```bash
chmod +x setup.sh flash.sh install_cubeprogrammer.sh
```

2. Optional environment bootstrap:

```bash
./setup.sh
```

3. Build firmware:

```bash
./flash.sh build
```

4. Flash over SWD (ST-Link):

```bash
./flash.sh flash
```

5. Build + flash in one command:

```bash
./flash.sh
```

## Flash Modes

### SWD (default)

```bash
./flash.sh flash
```

Script behavior:
- Tries `port=SWD` first
- Falls back to `port=USB1` DFU mode if SWD fails

### UART bootloader (CubeProgrammer)

```bash
./flash.sh serial /dev/ttyUSB0 115200
```

Notes:
- Set `BOOT0=1` before reset for system bootloader mode
- Typical serial parity is configured as EVEN in script

### UART bootloader (stm32flash, FlyMcu-style timing)

```bash
./flash.sh serial-flymcu /dev/ttyUSB0 115200
```

Requires:

```bash
sudo apt-get install -y stm32flash
```

## CMake Build (manual)

If you do not want to use `flash.sh`:

```bash
cmake -S . -B build -DCMAKE_TOOLCHAIN_FILE=arm-none-eabi-gcc.cmake -DCMAKE_BUILD_TYPE=Release
cmake --build build -j"$(nproc)"
```

Artifacts are generated in `build/`.

## Common Issues

1. `STM32_Programmer_CLI: command not found`
- Install CubeProgrammer and add its `bin` directory to `PATH`.

2. `No debug probe detected`
- Check ST-Link cable, udev permissions, and device power.

3. `Target device not found` in DFU mode
- Confirm BOOT pin state and USB DFU connection.

4. CMake cache points to old source path
- `flash.sh` auto-detects stale `build/CMakeCache.txt` and rebuilds `build/`.

5. ROS2 build errors in non-ASCII path
- If you build ROS2 interfaces in this project, use an ASCII-only path to avoid parser/path issues in some Humble setups.

## Clean

```bash
./flash.sh clean
```

## License

Please add your project license here.
