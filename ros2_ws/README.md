# aia_ws — 上位机 ROS2 工作空间

本仓库是配套 **stm-linux** 固件仓库的 ROS2 上位机工作空间，运行于 WSL2（Ubuntu 22.04 / ROS2 Humble）或树莓派。

## 整体架构

```
PC/WSL2 ─────────── 网线 ──────────── 树莓派 (192.168.10.2)
                                          │
                              /dev/ttyUSB0 → STM32（机械臂 DM4310+DM3510）
                              /dev/ttyACM0 → WHEELTEC 履带底盘
                              摄像头 → Flask 网页 → http://192.168.10.2:8080
```

- **WSL2**：编译固件（`stm-linux`）+ 运行 ROS2 节点（`aia_ws`）
- **树莓派**：直接运行控制脚本（`arm_car_control.py`），无需 ROS

## ROS2 包一览

| 包名 | 职责 | 运行位置 |
|---|---|---|
| `wheeltec_bridge` | 底盘控制：`/cmd_vel` → 串口 + 键盘节点 + RViz 可视化 | 树莓派 或 WSL |
| `stm32_robot_bridge` | 底盘状态接收：树莓派 UDP 转发 → `/imu/data` `/odom` `/tf` | WSL |
| `chassis_control` | CAN 差速控制节点（参考实现，通过 vcan0/can0 直驱电机） | WSL/Pi |

## 1. 环境准备

### 安装 ROS2 Humble（WSL2 / Ubuntu 22.04）

```bash
sudo apt install -y ros-humble-desktop python3-colcon-common-extensions
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 安装依赖

```bash
sudo apt install -y python3-serial
```

## 2. 构建

```bash
cd ~/aia_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

构建单个包：

```bash
colcon build --packages-select wheeltec_bridge
colcon build --packages-select stm32_robot_bridge
```

## 3. 使用方式

### wheeltec_bridge — 底盘 ROS2 控制

**树莓派**（启动串口节点，接收 /cmd_vel 并转串口给底盘）：

```bash
cd ~/aia_ws && source install/setup.bash
ros2 launch wheeltec_bridge wheeltec_teleop.launch.py port:=/dev/ttyACM0
```

**键盘控制**（在树莓派或 WSL 上）：

```bash
ros2 run wheeltec_bridge wheeltec_keyboard
```

**RViz 查看底盘模型**（WSL 侧）：

```bash
ros2 launch wheeltec_bridge view_chassis.launch.py
```

---

### stm32_robot_bridge — 底盘状态接入 ROS

**树莓派侧**先启动 UDP 转发脚本（位于 stm-linux 仓库）：

```bash
python3 ~/stm-linux/pi_chassis_udp_forwarder.py --serial-port /dev/ttyACM0 --wsl-ip 192.168.10.1 --udp-port 15050
```

**WSL 侧**启动桥接节点：

```bash
ros2 launch stm32_robot_bridge robot_bridge.launch.py udp_port:=15050
```

发布话题：`/imu/data`  `/odom`  `/tf`  `/chassis/battery_voltage`

---

### chassis_control — CAN 差速控制（参考实现）

通过 SocketCAN 直接控制达妙电机（差速底盘或调试）：

```bash
ros2 run chassis_control logic_node
```

> 默认连接 `vcan0`，实机切换为 `can0`。电机 ID1=左轮，ID2=右轮。

---

## 4. 树莓派一键配置

```bash
# 拷贝包到树莓派
scp -r ~/aia_ws/src/wheeltec_bridge ubuntu@192.168.10.2:~/

ssh ubuntu@192.168.10.2

# 网络配置（只需一次）
bash ~/wheeltec_bridge/setup_rpi_network.sh

# 安装依赖
sudo apt install -y python3-serial python3-flask python3-opencv python3-picamera2

# 构建
mkdir -p ~/aia_ws/src && mv ~/wheeltec_bridge ~/aia_ws/src/
cd ~/aia_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select wheeltec_bridge
source install/setup.bash
```

## 5. 网络配置

| 设备 | 接口 | IP |
|---|---|---|
| PC / WSL2 | 有线网卡 | `192.168.10.1/24` |
| 树莓派 | eth0 | `192.168.10.2/24` |

WSL2 每次启动需设置：

```bash
export ROS_DOMAIN_ID=10
source /opt/ros/humble/setup.bash
```

## 6. 配套仓库

- **stm-linux**：STM32 固件（CMake + arm-none-eabi-gcc）+ 树莓派控制脚本（`arm_car_control.py` 等）

详见 stm-linux 仓库 `README.md`。
