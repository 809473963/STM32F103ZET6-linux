# firmware — STM32 固件 + 控制脚本

本目录包含 STM32F103ZET6 固件源码、构建烧录脚本，以及树莓派和 PC 侧的控制/调试脚本。

---

## 目录结构

```
firmware/
├── CMakeLists.txt              ← 固件构建入口
├── arm-none-eabi-gcc.cmake     ← 交叉编译工具链
├── STM32F103ZETx_FLASH.ld      ← 链接脚本（512KB Flash）
├── flash.sh                    ← 构建/烧录一体化脚本 ← 主入口
├── setup.sh                    ← Linux 开发环境安装
├── install_cubeprogrammer.sh   ← STM32CubeProgrammer 安装
├── rpi_flash_stm32.sh          ← 通过树莓派 SSH 远程烧录
├── wsl_stm32_autoload.sh       ← WSL CH340 驱动自动挂载
│
├── CORE/           ← Cortex-M3 内核文件 + GCC 启动文件
├── SYSTEM/         ← 公共系统组件（delay / uart / sys）
├── STM32F10x_FWLib/← ST 标准外设库
├── USER/           ← STM32 配置头文件 + 中断服务
├── USER_ARM_DM/    ← 当前主应用 main.c（机械臂 DM 控制）
│
├── HARDWARE/
│   ├── DM_MULTI/   ← 达妙多电机 CAN 驱动（核心）
│   ├── DM4310/     ← DM4310 MIT 模式辅助函数
│   ├── PROTOCOL/   ← PC↔STM32 串口二进制协议
│   ├── GRIPPER42/  ← 42步进夹爪驱动（预留）
│   ├── BEEP/
│   ├── KEY/
│   └── LED/
│
├── pi/             ← 树莓派上运行的脚本
│   ├── arm_car_control.py       ← 一体化控制（机械臂+底盘+摄像头）
│   ├── pi_serial_tcp_bridge.py  ← 串口→TCP 桥接（供 WSL 远程访问）
│   └── pi_chassis_udp_forwarder.py ← 底盘状态→UDP 转发（供 ROS2）
│
├── tools/          ← PC / WSL 调试工具
│   ├── pc_keyboard_control.py  ← 纯机械臂键盘控制（TCP 或直连）
│   ├── wsl_to_stm32.py         ← WSL 直连 STM32 串口测试
│   ├── dm3510_diag.py          ← DM3510 全面诊断
│   ├── dm3510_reg_tool.py      ← DM3510 寄存器读写
│   ├── dmh3510_spin.py         ← 单电机转速测试
│   ├── dm_uart_calib.py        ← 串口标定工具
│   ├── dm_usb_fix.py           ← USB/CAN 适配器修复
│   ├── dm_windows_fix.py       ← Windows USB 环境修复
│   ├── xcan_bridge.py          ← XCAN-USB → PTY → slcand 桥接
│   ├── xcan_to_vcan.py         ← XCAN-USB ↔ vcan0 双向桥接
│   └── dm4310_can_keyboard.cpp ← 独立 C++ CAN 键盘测试程序
│
└── docs/           ← 文档与参数数据
    ├── ARM_DM_CONTROL_GUIDE.md     ← DM 电机联调完整指南
    ├── GRIPPER_42_TB6600_ZEROING.md← 夹爪归零操作指南
    ├── skill.md                    ← WSL 串口配置技巧
    ├── dm4310_tune.csv             ← DM4310 调参记录
    └── 三款电机参数对比表.xlsx      ← DM4310/DM3510/M24 参数对比
```

---

## 1. 环境安装

### Ubuntu / WSL2

```bash
chmod +x setup.sh flash.sh install_cubeprogrammer.sh
./setup.sh
```

安装内容：`arm-none-eabi-gcc`、`cmake`、`ninja-build`、`stm32flash`

### STM32CubeProgrammer

从 [ST 官网](https://www.st.com/en/development-tools/stm32cubeprog.html) 下载 Linux 安装包，放入本目录后：

```bash
./install_cubeprogrammer.sh
source ~/.bashrc
STM32_Programmer_CLI --version  # 验证
```

---

## 2. 固件构建

```bash
./flash.sh build
# 产物：build/STM32_KEY_PROJECT.{elf,hex,bin}
```

手动 CMake 等效：

```bash
cmake -S . -B build \
  -DCMAKE_TOOLCHAIN_FILE=arm-none-eabi-gcc.cmake \
  -DCMAKE_BUILD_TYPE=Release -DARM_DM_APP=ON
cmake --build build -j$(nproc)
```

---

## 3. 烧录方式

| 方式 | 命令 | 前提 |
|---|---|---|
| **Pi 远程烧录**（推荐） | `./rpi_flash_stm32.sh --fw build/STM32_KEY_PROJECT.bin` | Pi 通过 USB 连接 STM32 |
| SWD（ST-Link） | `./flash.sh flash` | 需要 ST-Link 调试器 |
| UART FlyMcu 时序 | `./flash.sh serial-flymcu /dev/ttyUSB0` | CH340/CH9102 串口直连 |

> 烧录时 BOOT0=1 + 复位；烧录后 BOOT0=0 + 复位，进入应用模式。

---

## 4. 树莓派控制脚本（pi/）

依赖：`pip install pyserial`

### arm_car_control.py — 日常使用主入口

```bash
# SSH 登录树莓派后
cd ~/arm_car_robot/firmware/pi

# 基本启动（机械臂 + 底盘）
python3 arm_car_control.py

# 指定端口
python3 arm_car_control.py --arm-port /dev/ttyUSB0 --car-port /dev/ttyACM0

# 带摄像头（网页 http://192.168.10.2:8080）
python3 arm_car_control.py --enable-cam --cam-devices 0 --enable-csi

# 仅底盘
python3 arm_car_control.py --no-arm
```

**按键速查：**

| 按键 | 功能 |
|---|---|
| `A` / `D` | 机械臂反转 / 正转（长按保持） |
| `W` / `S` | 臂速度 +0.1 / -0.1 rad/s |
| `1` / `2` / `0` | 选电机 DM4310 / DM3510 / 全选 |
| `F` | 使能 / 失能 |
| `I/K/J/L` | 底盘前/后/左/右 |
| `T` / `V` | 撑脚伸出 / 收回 |
| `Space` | 全停 |
| `Q` | 退出 |

### 通信桥接脚本（长期运行）

```bash
# 串口→TCP（供 WSL 通过网络控制 STM32）
python3 pi_serial_tcp_bridge.py

# 底盘状态→UDP（供 WSL ROS2 接收 IMU/Odom）
python3 pi_chassis_udp_forwarder.py --serial-port /dev/ttyACM0 --wsl-ip 192.168.10.1 --udp-port 15050
```

---

## 5. 调试工具（tools/）

| 脚本 | 用途 | 运行位置 |
|---|---|---|
| `pc_keyboard_control.py` | 纯机械臂键盘控制，支持 TCP 和直连串口 | PC/WSL 或 Pi |
| `dm3510_diag.py` | DM3510 电压/温度/错误码全面诊断 | PC/WSL |
| `dm3510_reg_tool.py` | 读写任意寄存器（如切换 VEL 模式） | PC/WSL |
| `dmh3510_spin.py` | 单电机转速阶跃测试 | PC/WSL |
| `dm_uart_calib.py` | 串口波特率/标定验证 | PC/WSL |
| `dm_usb_fix.py` | XCAN/USB-CAN 适配器诊断修复 | PC/WSL |
| `dm_windows_fix.py` | Windows 驱动冲突修复 | Windows |
| `wsl_to_stm32.py` | WSL 通过串口直接发控制帧 | WSL |
| `xcan_bridge.py` | XCAN-USB → PTY → slcand 桥接（接入 can0） | PC/WSL |
| `xcan_to_vcan.py` | XCAN-USB ↔ vcan0 双向转发 | PC/WSL |
| `dm4310_can_keyboard.cpp` | 独立 C++ CAN 键盘测试（需 g++ + libsocketcan） | Linux |

**切换 DM3510 到 VEL 模式（首次使用必做）：**

```bash
python3 tools/dm3510_reg_tool.py --id 2 --write 0x0A 3
```

---

## 6. 文档（docs/）

| 文件 | 内容 |
|---|---|
| `ARM_DM_CONTROL_GUIDE.md` | DM 电机 ID 扫描/改写、联调、重力补偿调参完整流程 |
| `GRIPPER_42_TB6600_ZEROING.md` | 42步进夹爪归零操作步骤 |
| `skill.md` | WSL2 串口自动挂载、Pi 远程烧录、TCP 桥接等操作技巧 |
| `dm4310_tune.csv` | DM4310 调参 KP/KD 记录 |
| `三款电机参数对比表.xlsx` | DM4310 / DM3510 / M24 参数对比 |

---

## 7. ROS2 集成

ROS2 包位于同仓库 `../ros2_ws/`，详见根目录 `README.md`。

---

## 8. 常见问题

**`STM32_Programmer_CLI: command not found`**：重新运行 `./install_cubeprogrammer.sh && source ~/.bashrc`

**烧录报 `Failed to init device, timeout`**：BOOT0 未置 1 或未断电复位，脚本会自动提示手动步骤

**电机使能后不动（DM3510）**：需先执行 `python3 tools/dm3510_reg_tool.py --id 2 --write 0x0A 3` 切 VEL 模式

**串口无心跳**：确认 BOOT0=0 + 复位，CH9102 的 DTR 直连 NRST，打开串口时须设 `setDTR(False)`

**CMake 路径报错**：`./flash.sh clean && ./flash.sh build`
