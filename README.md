# 机械臂履带车机器人

> STM32F103ZE + 达妙电机 + WHEELTEC 履带底盘 + ROS2 Humble 全栈机器人项目

本项目将两自由度机械臂（达妙 DM4310 + DM3510）与 WHEELTEC C10B 履带底盘集成在一起，以树莓派为中间层，支持键盘直控、ROS2 网络控制和远程烧录。

---

## 目录

- [系统架构](#1-系统架构)
- [硬件清单](#2-硬件清单)
- [仓库结构](#3-仓库结构)
- [快速开始](#4-快速开始)
- [固件开发（firmware/）](#5-固件开发-firmware)
- [树莓派控制脚本](#6-树莓派控制脚本)
- [ROS2 上位机（ros2_ws/）](#7-ros2-上位机-ros2_ws)
- [电机参数参考](#8-电机参数参考)
- [串口通信协议](#9-串口通信协议)
- [网络配置](#10-网络配置)
- [Windows / WSL2 开发指南](#11-windows--wsl2-开发指南)
- [常见问题](#12-常见问题)

---

## 1. 系统架构

```
┌─────────────────────────────────────────────────────────────────┐
│  PC / WSL2  (192.168.10.1)                                      │
│  ┌──────────────────────────────────────────────┐               │
│  │ ROS2 Humble                                   │               │
│  │  wheeltec_bridge  ──/cmd_vel──►              │               │
│  │  stm32_robot_bridge ◄─UDP─────               │               │
│  │  chassis_control   ──CAN──►(vcan0)           │               │
│  └──────────────────────────────────────────────┘               │
└───────────────────────── 网线 (RJ45) ───────────────────────────┘
                                │
┌───────────────────────────────▼─────────────────────────────────┐
│  Raspberry Pi  (192.168.10.2 / Ubuntu 22.04)                    │
│                                                                   │
│  arm_car_control.py  ←── SSH 键盘控制 / 本地运行                │
│  pi_chassis_udp_forwarder.py  ──UDP──►  WSL ROS2                │
│  pi_serial_tcp_bridge.py      ──TCP──►  WSL PC                  │
│                                                                   │
│  /dev/ttyUSB0 (CH9102)  ──UART 115200──►  STM32F103ZE           │
│  /dev/ttyACM0 (CH9102)  ──UART 115200──►  WHEELTEC C10B 主板   │
└─────────────────────────────────────────────────────────────────┘
          │                              │
┌─────────▼──────────────┐   ┌──────────▼──────────────────────┐
│  STM32F103ZET6          │   │  WHEELTEC C10B STM32            │
│  (自研固件)              │   │  (厂家固件，单字节串口协议)     │
│                          │   │                                  │
│  CAN1 总线               │   │  左履带电机                     │
│  ├── DM4310  (ID=1)      │   │  右履带电机                     │
│  │   底部关节 MIT 模式    │   └──────────────────────────────┘
│  └── DM3510  (ID=2)      │
│      第一关节 VEL 模式    │
└──────────────────────────┘
```

**数据流向总结：**

| 路径 | 协议 | 用途 |
|---|---|---|
| PC ↔ Pi | Ethernet TCP/IP | SSH 控制、ROS2 DDS、UDP 状态回传 |
| Pi → STM32 | UART 115200 自定义二进制帧 | 机械臂运动指令下发 |
| STM32 → Pi | UART 115200 自定义二进制帧 | 电机状态心跳回传 |
| STM32 ↔ DM电机 | CAN1 1Mbps | MIT/VEL 模式控制帧 |
| Pi → 底盘 | UART 115200 单字节 | 方向/速度指令 |

---

## 2. 硬件清单

| 组件 | 型号 | 说明 |
|---|---|---|
| **主控 MCU** | STM32F103ZET6 | Cortex-M3, 512KB Flash, 64KB RAM |
| **机械臂底部关节** | 达妙 DM4310 | CAN ID=1, MIT 模式, 10:1 减速, 峰值 7 N·m |
| **机械臂第一关节** | 达妙 DM3510 | CAN ID=2, VEL 模式, 直驱, 峰值 0.45 N·m |
| **履带底盘** | WHEELTEC C10B | 内置 STM32 + CH9102 USB-Serial |
| **中间计算单元** | Raspberry Pi 4B | Ubuntu 22.04, 连接 STM32 + 底盘 |
| **USB-Serial 桥** | CH9102 / CH340 | STM32 烧录 + 运行通信 |
| **CAN 收发器** | 板载 | 连接 DM4310 + DM3510，需 120Ω 终端电阻 |

---

## 3. 仓库结构

```
arm_car_robot/
├── README.md                    ← 本文档（项目总览）
├── .gitignore
│
├── firmware/                    ← STM32 固件 + 控制脚本
│   ├── CMakeLists.txt           ← 固件构建入口（CMake）
│   ├── arm-none-eabi-gcc.cmake  ← 交叉编译工具链配置
│   ├── STM32F103ZETx_FLASH.ld   ← 链接脚本（512KB Flash）
│   ├── flash.sh                 ← 构建/烧录一体化脚本（主入口）
│   ├── setup.sh                 ← Linux 开发环境安装
│   ├── install_cubeprogrammer.sh← STM32CubeProgrammer 安装
│   ├── rpi_flash_stm32.sh       ← 通过 Pi SSH 远程烧录
│   ├── wsl_stm32_autoload.sh    ← WSL CH340 驱动自动挂载
│   │
│   ├── CORE/                    ← Cortex-M3 内核 + GCC 启动文件
│   ├── SYSTEM/                  ← 公共系统组件（delay/uart/sys）
│   ├── STM32F10x_FWLib/         ← ST 标准外设库
│   ├── USER/                    ← STM32 配置头文件 + 中断处理
│   ├── USER_ARM_DM/             ← 当前主应用 main.c
│   └── HARDWARE/
│       ├── DM_MULTI/            ←  达妙多电机 CAN 驱动（核心）
│       ├── DM4310/              ←  DM4310 MIT 模式辅助函数
│       ├── PROTOCOL/            ←  PC↔STM32 串口二进制协议
│       ├── GRIPPER42/           ←  42步进夹爪驱动（预留）
│       └── BEEP/ KEY/ LED/      ←  基础外设
│
│   ├── pi/                      ← 树莓派上运行的脚本
│   │   ├── arm_car_control.py   ←  一体化控制（机械臂+底盘+摄像头）← 推荐入口
│   │   ├── pi_serial_tcp_bridge.py ← 串口→TCP 桥（供 WSL 远程访问）
│   │   └── pi_chassis_udp_forwarder.py ← 底盘状态→UDP（供 ROS2）
│   │
│   ├── tools/                   ← PC / WSL 调试工具
│   │   ├── pc_keyboard_control.py   ← 纯机械臂键盘控制
│   │   ├── dm3510_diag.py           ← DM3510 全面诊断
│   │   ├── dm3510_reg_tool.py       ← DM3510 寄存器读写
│   │   ├── dmh3510_spin.py          ← 单电机转速测试
│   │   ├── dm_uart_calib.py         ← 串口标定工具
│   │   ├── dm_usb_fix.py / dm_windows_fix.py ← USB 修复
│   │   ├── xcan_bridge.py / xcan_to_vcan.py  ← XCAN 工具
│   │   ├── wsl_to_stm32.py          ← WSL 直连 STM32
│   │   └── dm4310_can_keyboard.cpp  ← C++ CAN 键盘测试
│   │
│   ├── docs/                    ← 文档与参数数据
│   │   ├── ARM_DM_CONTROL_GUIDE.md  ← DM 电机联调完整指南
│   │   ├── GRIPPER_42_TB6600_ZEROING.md ← 夹爪归零指南
│   │   ├── skill.md             ← WSL 串口配置技巧
│   │   ├── dm4310_tune.csv      ← DM4310 调参记录
│   │   └── 三款电机参数对比表.xlsx
│   └── README.md                ← 固件详细文档
│
└── ros2_ws/                     ← ROS2 上位机工作空间（WSL2 / Pi）
    ├── README.md                ← ROS2 详细文档
    └── src/
        ├── wheeltec_bridge/     ← 底盘 ROS2 控制包
        │   ├── serial_node.py   ←  /cmd_vel → 串口（底盘驱动）
        │   ├── keyboard_node.py ←  键盘 → /cmd_vel 发布
        │   ├── chassis_marker_node.py ← RViz 底盘可视化
        │   ├── launch/          ←  wheeltec_teleop / view_chassis
        │   ├── urdf/            ←  底盘 URDF 模型
        │   └── rviz/            ←  RViz 配置文件
        │
        ├── stm32_robot_bridge/  ← 底盘状态 UDP→ROS 桥接包
        │   └── src/chassis_state_udp_bridge_node.cpp
        │       # 接收 Pi UDP 转发 → 发布 /imu/data /odom /tf
        │
        └── chassis_control/     ← CAN 差速控制参考实现
            └── logic_node.py    ← /cmd_vel → SocketCAN → 达妙电机
```

---

## 4. 快速开始

> **前提**：树莓派与 PC 通过网线直连（或同一局域网），IP 分别为 `192.168.10.2` 和 `192.168.10.1`。

### 场景一：烧录固件（首次或更新）

```bash
# 在 PC/WSL2 上编译
cd firmware
./flash.sh build

# 通过树莓派 SSH 远程烧录（推荐，树莓派连接 STM32 USB）
./rpi_flash_stm32.sh --fw build/STM32_KEY_PROJECT.bin
```

### 场景二：键盘控制机械臂 + 底盘（日常使用）

```bash
# SSH 登录树莓派后运行一体化控制脚本
ssh ubuntu@192.168.10.2
python3 ~/arm_car_robot/firmware/pi/arm_car_control.py
```

| 按键 | 功能 |
|---|---|
| `A` / `D` | 机械臂反转 / 正转 |
| `1` / `2` | 选择 DM4310 / DM3510 |
| `I/K/J/L` | 底盘前进/后退/左转/右转 |
| `T` / `V` | 撑脚伸出 / 收回 |
| `Space` | 全停 |
| `Q` | 退出 |

### 场景三：ROS2 控制（可视化 / 导航扩展）

```bash
# 树莓派：启动底盘串口节点
cd ~/arm_car_robot/ros2_ws && source install/setup.bash
ros2 launch wheeltec_bridge wheeltec_teleop.launch.py port:=/dev/ttyACM0

# WSL2：启动底盘状态桥接（接收 IMU/Odom）
ros2 launch stm32_robot_bridge robot_bridge.launch.py udp_port:=15050

# WSL2：RViz 查看底盘模型
ros2 launch wheeltec_bridge view_chassis.launch.py
```

---

## 5. 固件开发（firmware/）

### 5.1 环境安装（Ubuntu / WSL2）

```bash
cd firmware
chmod +x setup.sh flash.sh install_cubeprogrammer.sh
./setup.sh
```

安装内容：`arm-none-eabi-gcc`、`cmake`、`ninja-build`、`stm32flash`

### 5.2 安装 STM32CubeProgrammer

从 [ST 官网](https://www.st.com/en/development-tools/stm32cubeprog.html) 下载 Linux 安装包，放入 `firmware/` 目录后：

```bash
./install_cubeprogrammer.sh
source ~/.bashrc
STM32_Programmer_CLI --version  # 验证
```

### 5.3 构建固件

```bash
./flash.sh build
# 产物：build/STM32_KEY_PROJECT.{elf,hex,bin}
```

手动 CMake 等效命令：

```bash
cmake -S . -B build \
  -DCMAKE_TOOLCHAIN_FILE=arm-none-eabi-gcc.cmake \
  -DCMAKE_BUILD_TYPE=Release -DARM_DM_APP=ON
cmake --build build -j$(nproc)
```

### 5.4 烧录方式

| 方式 | 命令 | 条件 |
|---|---|---|
| **Pi 远程烧录**（推荐） | `./rpi_flash_stm32.sh --fw build/STM32_KEY_PROJECT.bin` | Pi 通过 USB 连接 STM32 |
| SWD（ST-Link） | `./flash.sh flash` | 需要 ST-Link 调试器 |
| UART FlyMcu 时序 | `./flash.sh serial-flymcu /dev/ttyUSB0` | CH340/CH9102 串口直连 |

### 5.5 固件代码结构

```
USER_ARM_DM/main.c          ← 应用入口（Protocol_Init + 主循环）
HARDWARE/PROTOCOL/          ← 串口帧解析 + 电机指令分发 + 重力补偿
HARDWARE/DM_MULTI/          ← 达妙电机 CAN 驱动（MIT/VEL 模式）
HARDWARE/DM4310/            ← DM4310 MIT 模式辅助函数
```

---

## 6. 树莓派控制脚本

树莓派控制脚本位于 `firmware/pi/`，调试工具位于 `firmware/tools/`（均需 `pip install pyserial`）。

### 主控脚本：arm_car_control.py

一体化控制程序，支持机械臂 + 底盘 + 摄像头。

```bash
# 在树莓派上运行（cd 到 pi/ 目录）
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

### 调试工具脚本

| 脚本 | 用途 |
|---|---|
| `tools/dm3510_diag.py` | DM3510 全面诊断（电压/温度/错误码）|
| `tools/dm3510_reg_tool.py` | 读写 DM3510 任意寄存器 |
| `tools/dmh3510_spin.py` | 单电机转速测试 |
| `tools/dm_uart_calib.py` | 串口波特率/标定诊断 |
| `tools/pc_keyboard_control.py` | 纯机械臂键盘控制（TCP 或直连）|
| `tools/wsl_to_stm32.py` | WSL 直连 STM32 调试 |
| `tools/xcan_bridge.py` | XCAN USB-CAN 适配器桥接 |

### 通信桥接脚本（长期运行）

```bash
# Pi 串口→TCP（供 WSL 通过网络访问 STM32 串口）
python3 firmware/pi/pi_serial_tcp_bridge.py

# Pi 底盘状态→UDP（供 WSL ROS2 接收 IMU/Odom）
python3 firmware/pi/pi_chassis_udp_forwarder.py --serial-port /dev/ttyACM0 --wsl-ip 192.168.10.1 --udp-port 15050
```

---

## 7. ROS2 上位机（ros2_ws/）

运行环境：WSL2 / Ubuntu 22.04 + ROS2 Humble。

### 7.1 安装 ROS2

```bash
sudo apt install -y ros-humble-desktop python3-colcon-common-extensions
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 7.2 构建工作空间

```bash
cd ros2_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

### 7.3 各包说明

#### wheeltec_bridge（底盘控制）

将 ROS2 `/cmd_vel` 话题转换为 WHEELTEC 单字节串口协议，发送给底盘。

```bash
# 启动串口桥接节点
ros2 launch wheeltec_bridge wheeltec_teleop.launch.py port:=/dev/ttyACM0

# 启动键盘控制节点
ros2 run wheeltec_bridge wheeltec_keyboard

# RViz 可视化
ros2 launch wheeltec_bridge view_chassis.launch.py
```

发布话题：`/cmd_vel`（接收）、`/wheeltec/speed_set`（速度档位）

#### stm32_robot_bridge（底盘状态接入 ROS）

接收树莓派通过 UDP 转发的底盘状态，解析后发布标准 ROS2 话题。

```bash
ros2 launch stm32_robot_bridge robot_bridge.launch.py udp_port:=15050
```

发布话题：

| 话题 | 类型 | 内容 |
|---|---|---|
| `/imu/data` | `sensor_msgs/Imu` | IMU 姿态、加速度、角速度 |
| `/odom` | `nav_msgs/Odometry` | 底盘速度 + 积分位置 |
| `/tf` | TF | `odom → base_link` 变换 |
| `/chassis/battery_voltage` | `std_msgs/Float32` | 电池电压 |

#### chassis_control（CAN 差速控制参考实现）

通过 SocketCAN 直接向达妙电机发速度指令（差速底盘模型参考）。

```bash
ros2 run chassis_control logic_node  # 默认连接 vcan0，实机改为 can0
```

---

## 8. 电机参数参考

| 参数 | DM4310（底部关节） | DM3510（第一关节） |
|---|---|---|
| CAN ID | 1 (0x01) | 2 (0x02) |
| Feedback ID | 0x11 | 0x12 |
| 控制模式 | MIT（位置+速度+力矩） | VEL（速度环） |
| 减速比 | 10:1 | 1:1（直驱） |
| 峰值力矩 | 7 N·m | 0.45 N·m |
| 位置范围 (q_max) | ±12.5 rad | ±12.5 rad |
| 速度范围 (dq_max) | 30 rad/s | 280 rad/s |
| CAN 控制帧 ID | 0x001 | 0x202（VEL: slave_id+0x200） |

### DM 电机使能流程

**DM4310（MIT 模式）**：
```
Disable → Enable → SetMIT(p, v, kp, kd, t_ff)
```

**DM3510（VEL 模式）**：
```
Disable → WriteReg(0x0A=3) → Enable → SetVel(rad_s)
```

### MIT 控制帧格式（DM4310）

```
CAN ID = 0x001
Data[8]: [p_des(16bit)][v_des(12bit)][kp(12bit)][kd(12bit)][t_ff(12bit)]
```

---

## 9. 串口通信协议

PC/Pi 与 STM32 之间使用自定义二进制帧协议（UART1，115200 bps）。

### 帧格式

```
 0xAA | LEN | CMD | DATA[LEN] | CKSUM
```

- `0xAA`：帧头（固定）
- `LEN`：数据字节数（不含帧头/CMD/CKSUM）
- `CMD`：命令字节
- `CKSUM`：所有字节（帧头到 DATA 最后一字节）的字节和低 8 位

### 主要命令

| CMD | 说明 | 数据格式 |
|---|---|---|
| `0x01` | 设置电机速度（MIT 模式） | `[motor_id][p_des(f32)][v_des(f32)][kp(f32)][kd(f32)][t_ff(f32)]` |
| `0x02` | 全部停止失能 | 无 |
| `0x03` | 请求状态回复 | 无 |
| `0x04` | 设置电机使能/失能 | `[motor_id][enable(0/1)]` |
| `0x05` | 设置电机速度（VEL 模式，mrad/s × 1000） | `[motor_id][vel(i32)]` |
| `0x20` | 网关帧（MIT 透传） | 完整 MIT 控制结构 |
| `0x21` | 扫描 CAN 总线电机 ID | 无 |
| `0x24` | 设置重力补偿参数 | `[motor_id][kg(f32)][bias(f32)]` |
| `0x83` | **STM32 → PC** 状态心跳 | `[motor_id][pos(f32)][vel(f32)][tau(f32)][err][temps]` |

> **安全看门狗**：500 ms 内未收到合法帧，STM32 自动 `arm_stop_all()` 失能所有电机。

---

## 10. 网络配置

| 设备 | 接口 | IP 地址 |
|---|---|---|
| PC / WSL2 | 有线网卡 | `192.168.10.1 / 24` |
| 树莓派 | eth0 | `192.168.10.2 / 24` |

### PC 端设置

**Windows（通过 WSL2）**：
- 有线网卡 → IPv4 → 手动：IP `192.168.10.1`，子网 `255.255.255.0`

**WSL2 每次启动**：
```bash
export ROS_DOMAIN_ID=10
source /opt/ros/humble/setup.bash
```

**Ubuntu PC**：
```bash
sudo nmcli con modify "有线连接" ipv4.method manual ipv4.addresses 192.168.10.1/24
echo "export ROS_DOMAIN_ID=10" >> ~/.bashrc
```

### 树莓派网络配置（一次性）

```bash
bash ~/arm_car_robot/ros2_ws/src/wheeltec_bridge/setup_rpi_network.sh
```

---

## 11. Windows / WSL2 开发指南

### 所需工具

| 工具 | 获取方式 | 用途 |
|---|---|---|
| **WSL2 + Ubuntu 22.04** | `wsl --install -d Ubuntu-22.04` | Linux 开发环境 |
| **USBIPD-WIN** | [GitHub releases](https://github.com/dorssel/usbipd-win/releases) `.msi` | USB 设备透传到 WSL2 |
| **arm-none-eabi-gcc** | `apt install gcc-arm-none-eabi` | 固件交叉编译 |
| **STM32CubeProgrammer** | [ST 官网](https://www.st.com/en/development-tools/stm32cubeprog.html) Linux 版 | SWD/UART 烧录 |

### 每次插入 USB 设备后

在 Windows PowerShell（管理员）：

```powershell
usbipd list                          # 查找 ST-Link 或 CH340 的 BUSID
usbipd bind --busid <BUSID>
usbipd attach --wsl --busid <BUSID>
```

在 WSL2 验证：

```bash
ls /dev/ttyUSB* /dev/ttyACM*
```

### WSL2 CH340 自动挂载（一次性）

```bash
cd firmware
sudo install -m 755 wsl_stm32_autoload.sh /usr/local/bin/
printf '[boot]\ncommand=/usr/local/bin/wsl_stm32_autoload.sh\n' | sudo tee /etc/wsl.conf
```

```powershell
# Windows PowerShell
wsl --shutdown
```

---

## 12. 常见问题

### Q: STM32_Programmer_CLI: command not found

重新安装并刷新 PATH：

```bash
cd firmware && ./install_cubeprogrammer.sh && source ~/.bashrc
```

### Q: 树莓派烧录报 `Failed to init device, timeout`

- BOOT0 未拨到 1，或未断电复位
- 脚本会自动提示手动操作步骤

### Q: 电机使能后不动（DM3510）

DM3510 首次使用需先切换到 VEL 模式：

```bash
python3 firmware/tools/dm3510_reg_tool.py --id 2 --write 0x0A 3
```

### Q: STM32 烧录成功但串口无心跳输出

确认 BOOT0 已拨回 0 并复位，使用以下命令监听：

```bash
python3 - <<'PY'
import serial, time
s = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.2)
s.dsrdtr = False; s.open() if not s.is_open else None
s.setDTR(False); s.setRTS(False)
t0 = time.time()
while time.time()-t0 < 5:
    b = s.read(32)
    if b: print(b.hex(' '))
PY
```

### Q: ROS2 话题跨机器看不到

检查两台机器的 `ROS_DOMAIN_ID` 是否相同（均设为 `10`），且网络能互通（`ping 192.168.10.2`）。

### Q: 构建后 CMake 报路径错误

```bash
cd firmware && ./flash.sh clean && ./flash.sh build
```

`flash.sh` 会自动检测并清理失效的 CMake 缓存。

---

## License

本项目尚未明确 License，准备开源发布时请补充 `LICENSE` 文件。
