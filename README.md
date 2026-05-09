# STM32F103ZET6 机械臂 + 底盘控制（初版）

基于 Linux 的 STM32F103ZET6 全电机控制工程，`main` 为**初版完整配置**，`simplified` 为最终降级成品。

> **降级简化版**（仅 DM4310+DM3510 两轴 + WHEELTEC 底盘）见 [`simplified` 分支](../../tree/simplified)。

---

## 硬件配置

| 硬件 | 数量 | 驱动方式 |
|---|---|---|
| STM32F103ZET6 | 1 | 主控，USART 115200 |
| DM4310 无刷电机 | 1 | CAN MIT 模式（底部关节） |
| 86 步进电机 | 2 | 脉冲/方向，MOTOR86 驱动 |
| 直流编码器电机 | 4 | BTS7960 H桥 + 编码器反馈 |

---

## 目录说明

```
.
├── CORE/                   ← Cortex-M3 内核 + GCC 启动文件
├── SYSTEM/                 ← 公共组件（delay/uart/sys）
├── STM32F10x_FWLib/        ← ST 标准外设库
├── USER/                   ← STM32 配置头文件 + 中断
├── USER_ENCODER_MOTOR/     ← 编码器电机应用入口
├── USER_MOTOR86/           ← 86步进电机应用入口
├── USER_HELLO/             ← 最小验证应用
│
├── HARDWARE/
│   ├── DM4310/             ← DM4310 CAN MIT 驱动
│   ├── ENCODER_MOTOR/      ← 直流编码器电机驱动
│   ├── MOTOR/              ← BTS7960 PWM 驱动
│   ├── MOTOR86/            ← 86步进脉冲/方向驱动
│   ├── PROTOCOL/           ← PC↔STM32 串口二进制协议
│   └── BEEP/ KEY/ LED/
│
├── ros2_ws/                ← ROS2 完整工作空间（见 ros2_ws/README.md）
│   ├── stm32_robot_bridge/ ← ① 机械臂串口桥接+键控  ② 底盘状态 UDP→ROS
│   ├── stm32_serial_bridge/← DM4310 参数 tkinter 实时调节 GUI
│   ├── wheeltec_bridge/    ← WHEELTEC C10B 底盘串口控制 + RViz 可视化
│   └── chassis_control/    ← CAN 差速控制（DM4310 MIT + DM3510 VEL）
│
├── images/                 ← 硬件照片、接线图
│
├── CMakeLists.txt          ← 固件构建入口
├── arm-none-eabi-gcc.cmake ← 交叉编译工具链
├── STM32F103ZETx_FLASH.ld  ← 链接脚本（512KB Flash）
├── flash.sh                ← 构建/烧录一体化脚本
├── setup.sh                ← Linux 环境安装
├── install_cubeprogrammer.sh
├── rpi_flash_stm32.sh      ← 通过树莓派 SSH 远程烧录
├── wsl_stm32_autoload.sh   ← WSL CH340 自动挂载
├── pc_keyboard_control.py  ← PC 键盘控制脚本（编码器电机）
├── dm4310_can_keyboard.cpp ← DM4310 独立 CAN 测试（C++）
└── wsl_to_stm32.py         ← WSL 直连 STM32 调试
```

---

## 环境准备

### Ubuntu / WSL2

```bash
chmod +x setup.sh flash.sh install_cubeprogrammer.sh
./setup.sh
```

安装内容：`arm-none-eabi-gcc`、`cmake`、`ninja-build`、`stm32flash`

### STM32CubeProgrammer

```bash
./install_cubeprogrammer.sh
source ~/.bashrc
STM32_Programmer_CLI --version  # 验证
```

---

## 固件构建

```bash
# 编码器电机应用
./flash.sh encoder-build

# 86步进电机应用
./flash.sh motor86-build

# 最小 Hello 测试
./flash.sh hello-build
```

产物位于对应 `build_encoder/`、`build_motor86/`、`build_hello/` 目录（已加入 .gitignore）。

---

## 烧录

| 方式 | 命令 | 前提 |
|---|---|---|
| **Pi 远程烧录**（推荐） | `./rpi_flash_stm32.sh` | Pi 通过 USB 连接 STM32 |
| SWD（ST-Link） | `./flash.sh flash` | 需要 ST-Link 调试器 |
| UART FlyMcu | `./flash.sh serial-flymcu /dev/ttyUSB0` | CH340 串口直连 |
| 编码器应用 UART | `sudo ./flash.sh encoder-serial-flymcu /dev/ttyUSB0` | 同上 |

> 烧录时 BOOT0=1 + 复位；烧录后 BOOT0=0 + 复位，进入应用。

---

## 键盘控制（编码器电机，无 ROS）

```bash
python3 -m pip install --user pyserial
python3 pc_keyboard_control.py --port /dev/ttyUSB0 --baud 115200 --motor 1
```

| 按键 | 功能 |
|---|---|
| `W` / `S` | 加速 / 减速 |
| `A` / `D` | 反转 / 正转 |
| `Space` | 急停 |
| `Q` | 退出 |

---

## ROS2 工作空间

完整说明见 [`ros2_ws/README.md`](ros2_ws/README.md)，本工作空间包含四个包：

| 包 | 节点 | 用途 |
|---|---|---|
| `stm32_robot_bridge` | `serial_bridge_node`<br>`teleop_keyboard_node`<br>`chassis_state_udp_bridge_node` | 机械臂串口控制（初版）<br>键盘 → MotorCommand<br>底盘状态 UDP→ROS |
| `stm32_serial_bridge` | `tuning_slider_gui` | DM4310 PD 参数实时调节 GUI |
| `wheeltec_bridge` | `serial_node` `keyboard_node` `chassis_marker_node` | WHEELTEC 底盘控制 + RViz |
| `chassis_control` | `logic_node` | CAN 差速控制（简化版硬件适用） |

### 构建

```bash
cd ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
```

### 机械臂键盘控制（初版 encoder-build 固件）

```bash
# launch 文件一键启动（串口桥接 + 键盘控制）
ros2 launch stm32_robot_bridge robot_bridge.launch.py

# 或分开启动
ros2 run stm32_robot_bridge serial_bridge_node \
  --ros-args -p port_name:=/dev/ttyUSB0 -p baud_rate:=115200
ros2 run stm32_robot_bridge teleop_keyboard_node
```

### DM4310 参数调节 GUI

```bash
ros2 launch stm32_serial_bridge stm32_tuning_gui.launch.py
```

### WHEELTEC 底盘键盘遥控

```bash
ros2 launch wheeltec_bridge wheeltec_teleop.launch.py
```

### 底盘状态接入 ROS（配合树莓派 UDP 转发）

```bash
# 树莓派侧（先运行）
python3 pi_chassis_udp_forwarder.py --serial-port /dev/ttyACM0 \
  --wsl-ip 192.168.10.1 --udp-port 15050

# WSL 侧
ros2 run stm32_robot_bridge chassis_state_udp_bridge_node \
  --ros-args -p udp_port:=15050
```

> 确保两端 `export ROS_DOMAIN_ID=<相同值>`，详见 `ros2_ws/README.md`。

---

## 分支说明

| 分支 | 说明 |
|---|---|
| `main` | **初版完整配置**：DM4310 + 86步进×2 + 编码器电机×4；ros2_ws 含全部四个包 |
| `simplified` | **最终成品**：仅 DM4310(MIT)+DM3510(VEL)；固件按 firmware/pi/tools/docs 重组 |

---

## 常见问题

**串口权限不足**：`sudo usermod -aG dialout $USER`，重新登录生效

**构建缓存过期**：`./flash.sh clean && ./flash.sh encoder-build`

**DM4310 不响应**：确认 CAN 接线（PA12 TX / PA11 RX）、终端电阻（120Ω）、共地

**ROS2 找不到 motor_control_msgs**：如不使用 `stm32_serial_bridge` 可跳过：
```bash
colcon build --packages-ignore stm32_serial_bridge
```
