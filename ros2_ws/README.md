# ros2_ws — ROS2 完整工作空间

本工作空间包含机械臂串口控制、底盘驱动、参数调节 GUI 共四个包，覆盖初版（多电机）与简化版（仅 DM）硬件配置。

---

## 包总览

| 包名 | 类型 | 用途 |
|---|---|---|
| `stm32_robot_bridge` | C++ | STM32 串口↔ROS2 话题桥接（机械臂键控 + 底盘状态 UDP→ROS） |
| `stm32_serial_bridge` | Python | DM4310 参数实时调节 tkinter GUI |
| `wheeltec_bridge` | Python | WHEELTEC C10B 底盘串口控制 + 键盘 + RViz 可视化 |
| `chassis_control` | Python | CAN 差速控制节点（DM4310+DM3510，仅简化版适用） |

---

## 依赖安装

```bash
sudo apt install python3-colcon-common-extensions
cd ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

## 构建

```bash
cd ros2_ws
colcon build
source install/setup.bash
```

构建单个包：
```bash
colcon build --packages-select stm32_robot_bridge
colcon build --packages-select wheeltec_bridge
```

---

## 包详细说明

---

### 1. stm32_robot_bridge

**两种节点，分别对应两套硬件：**

#### 1a. 机械臂串口桥接（初版）

面向：编码器电机 + BTS7960 + DM4310，PC 通过串口直连 STM32。

节点：
- **`serial_bridge_node`**：订阅 `/motor_commands` → 串口下发 STM32 控制帧
- **`teleop_keyboard_node`**：键盘 → 发布 `/motor_commands`

自定义消息：
```
MotorCommand.msg   : motor_id(int32), command(string), value(float32)
MotorStatus.msg    : motor_id(int32), position(float32), velocity(float32)
```

启动：
```bash
# launch 文件（同时启动两个节点）
ros2 launch stm32_robot_bridge robot_bridge.launch.py

# 或分开启动
ros2 run stm32_robot_bridge serial_bridge_node \
  --ros-args -p port_name:=/dev/ttyUSB0 -p baud_rate:=115200

ros2 run stm32_robot_bridge teleop_keyboard_node
```

键盘控制（`teleop_keyboard_node`）：

| 按键 | 功能 |
|---|---|
| `W` / `S` | 加速 / 减速 |
| `A` / `D` | 反转 / 正转 |
| `1`–`9` | 速度档位 10%–90% |
| `Space` | 急停 |

---

#### 1b. 底盘状态 UDP→ROS 桥接（简化版适用）

面向：树莓派 `pi_chassis_udp_forwarder.py` 将底盘状态 UDP 转发到 WSL2。

节点：
- **`chassis_state_udp_bridge_node`**：接收 UDP 数据包 → 发布 `/odom`、`/imu`、`/battery_voltage`

启动：
```bash
ros2 run stm32_robot_bridge chassis_state_udp_bridge_node \
  --ros-args -p udp_port:=15050
```

> 树莓派侧需先运行：
> ```bash
> python3 pi/pi_chassis_udp_forwarder.py --serial-port /dev/ttyACM0 \
>   --wsl-ip 192.168.10.1 --udp-port 15050
> ```

---

### 2. stm32_serial_bridge

DM4310 MIT 模式参数实时调节 GUI（基于 tkinter + ROS2 `SetParameters` 服务）。

```bash
ros2 launch stm32_serial_bridge stm32_tuning_gui.launch.py
```

- 滑条覆盖 PD 参数 `p_des`、`v_des`、`kp`、`kd`、`t_ff`
- 实时下发到固件参数服务器
- 适合调节 DM4310 MIT 控制的刚度/阻尼

---

### 3. wheeltec_bridge

WHEELTEC C10B 履带底盘控制包，包含三个节点：

| 节点 | 说明 |
|---|---|
| `serial_node` | `/cmd_vel` → 串口二进制帧 → WHEELTEC 底盘 |
| `keyboard_node` | 键盘输入 → 发布 `/cmd_vel` |
| `chassis_marker_node` | RViz2 底盘模型可视化 |

启动：
```bash
# 键盘遥控底盘
ros2 launch wheeltec_bridge wheeltec_teleop.launch.py

# 仅 RViz 可视化
ros2 launch wheeltec_bridge view_chassis.launch.py
```

键盘控制（`keyboard_node`）：

| 按键 | 功能 |
|---|---|
| `W` / `X` | 前进 / 后退 |
| `A` / `D` | 左转 / 右转 |
| `S` | 原地停止 |
| `Q` / `E` | 左前 / 右前斜行 |
| `Z` / `C` | 左后 / 右后斜行 |
| `+` / `-` | 速度上限 ±0.1 m/s |
| `Space` | 急停 |

节点参数（`serial_node`）：
```yaml
serial_port: /dev/ttyACM0
baud_rate: 115200
```

Pi 端配网脚本：
```bash
bash ros2_ws/src/wheeltec_bridge/setup_rpi_network.sh
```

详细 Pi 配置见 `src/wheeltec_bridge/WHEELTEC_RPI_GUIDE.md`。

---

### 4. chassis_control

CAN 差速控制节点，仅适用于**简化版硬件**（DM4310 MIT + DM3510 VEL）。

订阅 `/cmd_vel`（`geometry_msgs/Twist`），换算为左右轮 CAN 速度指令，超时自动停机。

```bash
ros2 run chassis_control logic_node
```

参数：
```
wheel_separation: 0.3      # 轮距 (m)
wheel_radius: 0.075        # 轮半径 (m)
watchdog_timeout: 0.5      # 指令超时停机 (s)
```

> 需要底层 `socketcan`：`sudo ip link set can0 up type can bitrate 1000000`

---

## 话题汇总

```
/cmd_vel            ← keyboard_node / 外部发布  →  serial_node / chassis_control
/motor_commands     ← teleop_keyboard_node      →  serial_bridge_node
/odom               ← chassis_state_udp_bridge_node (底盘编码器积分)
/imu                ← chassis_state_udp_bridge_node (底盘 IMU)
/battery_voltage    ← chassis_state_udp_bridge_node
/chassis_marker     ← chassis_marker_node       →  RViz2
```

---

## 常见问题

**`colcon build` 报找不到 `motor_control_msgs`**：仅在使用 `stm32_serial_bridge` 时需要，如不使用可跳过该包：
```bash
colcon build --packages-ignore stm32_serial_bridge
```

**串口权限**：`sudo usermod -aG dialout $USER`，重新登录生效

**ROS_DOMAIN_ID 不一致**：确保 PC 和 Pi 的 `ROS_DOMAIN_ID` 相同：
```bash
export ROS_DOMAIN_ID=42   # 两端保持一致
```
