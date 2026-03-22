# STM32F103ZET6 Keil → Linux 迁移 + ROS2 机器人控制框架

将正点原子 STM32F103ZET6 项目从 Keil MDK 迁移到 Linux (arm-none-eabi-gcc + CMake)，使用 **STM32CubeProgrammer** 烧录，集成 ROS2 实现 **键盘遥控多电机机器人**。

## 现有项目分析

| 项目 | 详情 |
|------|------|
| MCU | STM32F103ZE (Cortex-M3, 72MHz) |
| Flash / RAM | 512KB @ 0x08000000 / 64KB @ 0x20000000 |
| Keil 宏定义 | `STM32F10X_HD`, `USE_STDPERIPH_DRIVER` |
| 连接方式 | **USB**（USB 转串口，通过 ST-Link 板载） |
| 烧录工具 | **STM32CubeProgrammer** (`STM32_Programmer_CLI`) |
| 已安装 | ROS2 Humble |

---

## Proposed Changes

### 1. 构建系统

#### [NEW] [CMakeLists.txt](file:///home/luo/实验2 按键输入/CMakeLists.txt)
- 交叉编译，Cortex-M3 编译选项
- **自动扫描**源文件（增删 .c 文件无需改 CMake）
- 输出 `.elf` / `.hex` / `.bin`
- `make flash` 目标调用 STM32CubeProgrammer

#### [NEW] [arm-none-eabi-gcc.cmake](file:///home/luo/实验2 按键输入/arm-none-eabi-gcc.cmake)
CMake 工具链文件。

#### [NEW] [STM32F103ZETx_FLASH.ld](file:///home/luo/实验2 按键输入/STM32F103ZETx_FLASH.ld)
GCC 链接脚本（512KB Flash + 64KB RAM）。

#### [NEW] [CORE/startup_stm32f103xe_gcc.s](file:///home/luo/实验2 按键输入/CORE/startup_stm32f103xe_gcc.s)
GNU AS 格式启动文件（替代 Keil ARM 汇编语法）。

---

### 2. 一键编译 + 烧录

#### [NEW] [flash.sh](file:///home/luo/实验2 按键输入/flash.sh)
```bash
./flash.sh          # 编译 + 烧录（USB/SWD）
./flash.sh build    # 仅编译
./flash.sh flash    # 仅烧录
./flash.sh clean    # 清理
```
使用 `STM32_Programmer_CLI -c port=SWD -w firmware.bin 0x08000000 -rst` 烧录。

#### [NEW] [setup.sh](file:///home/luo/实验2 按键输入/setup.sh)
安装 `gcc-arm-none-eabi`、`cmake`。提示用户手动安装 STM32CubeProgrammer。

---

### 3. STM32 端 — 电机驱动 & 通信协议

#### [NEW] [HARDWARE/MOTOR/motor.h](file:///home/luo/实验2 按键输入/HARDWARE/MOTOR/motor.h) / [motor.c](file:///home/luo/实验2 按键输入/HARDWARE/MOTOR/motor.c)
电机驱动抽象层：
- `Motor_Init()` — 初始化 PWM 定时器 + GPIO
- `Motor_SetSpeed(id, speed, dir)` — 控制单个电机
- `Motor_StopAll()` — 急停
- 支持 4 路电机（可扩展）

#### [NEW] [HARDWARE/PROTOCOL/protocol.h](file:///home/luo/实验2 按键输入/HARDWARE/PROTOCOL/protocol.h) / [protocol.c](file:///home/luo/实验2 按键输入/HARDWARE/PROTOCOL/protocol.c)
USB 串口通信协议：
- 帧格式：`[0xAA] [LEN] [CMD] [DATA...] [CHECKSUM]`
- 命令：设置电机速度、查询状态、急停

---

### 4. ROS2 — 键盘遥控 + 串口桥接

#### [NEW] ros2_ws/src/stm32_robot_bridge/
```
stm32_robot_bridge/
├── CMakeLists.txt / package.xml
├── msg/
│   ├── MotorCommand.msg        # motor_id, speed, direction
│   └── MotorStatus.msg         # motor_id, current_speed, status
├── src/
│   ├── serial_bridge_node.cpp  # USB 串口 ↔ ROS2 话题桥接
│   └── teleop_keyboard_node.cpp # 键盘控制节点（核心）
├── launch/robot_bridge.launch.py
└── config/serial_config.yaml   # USB 串口设备 & 波特率
```

**teleop_keyboard_node** — 键盘遥控：
- `W/S` — 前进/后退
- `A/D` — 左转/右转
- `Q/E` — 原地左旋/右旋
- `Space` — 急停
- `1-9` — 速度档位
- 发布 `MotorCommand` 话题，由 `serial_bridge_node` 转发到 STM32

---

## 最终项目结构
```
实验2 按键输入/
├── CMakeLists.txt / arm-none-eabi-gcc.cmake / STM32F103ZETx_FLASH.ld
├── flash.sh / setup.sh
├── CORE/  (+ startup_stm32f103xe_gcc.s)
├── HARDWARE/ (LED/ KEY/ BEEP/ + MOTOR/ + PROTOCOL/)
├── SYSTEM/ STM32F10x_FWLib/ USER/    ← 保留不变
├── build/                             ← 编译输出
└── ros2_ws/src/stm32_robot_bridge/    ← ROS2 键盘遥控
```

## Verification Plan
1. `setup.sh` 安装工具链 → CMake 编译通过生成 `.bin`
2. `./flash.sh` 烧录验证（需 STM32CubeProgrammer + 硬件）
3. `colcon build` 编译 ROS2 功能包
4. `ros2 run stm32_robot_bridge teleop_keyboard_node` 键盘控制测试

## Verification Result (2026-03-16)
1. `./flash.sh build`：通过，`STM32_KEY_PROJECT.elf` 成功生成。
2. 修复 `flash.sh`：当 `build/CMakeCache.txt` 的源目录与当前工程目录不一致时，自动重建 `build/`，解决目录迁移（中文路径 → 当前路径）导致的构建失败。
3. ROS2 包在中文路径下构建失败：`Target dependency '2_' does not exist`（`rosidl_generate_interfaces` 路径解析兼容问题）。
4. 将 ROS2 包复制到 ASCII 路径 `/tmp/stm32_ros2_ascii` 后：
	- `colcon build --packages-select stm32_robot_bridge` 通过。
	- `ros2 run stm32_robot_bridge serial_bridge_node` 可启动。
	- `ros2 run stm32_robot_bridge teleop_keyboard_node` 可启动。
5. 烧录前置检查：`STM32_Programmer_CLI` 当前未在 PATH 中（`command -v STM32_Programmer_CLI` 无输出），因此暂未完成硬件烧录验证。
6. `./flash.sh flash` 行为验证：在未安装 CLI 的情况下会明确报错并退出，不会误执行烧录流程。
7. 已安装 STM32CubeProgrammer（CLI 版本 `2.22.0`），并将路径加入 `~/.bashrc`。
8. 重新执行 `./flash.sh flash`：脚本已能正常调用 CLI；当前失败原因为硬件未连接（`No debug probe detected`，SWD 失败后 DFU 报 `Target device not found`）。
