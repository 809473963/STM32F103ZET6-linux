# ros2_ws — ROS2 机械臂控制工作空间（原始版）

本目录为配套 STM32F103ZET6 固件的 ROS2 上位机工作空间，面向**初版机械臂**硬件配置：

| 硬件 | 数量 | 控制方式 |
|---|---|---|
| DM4310 无刷电机 | 1 | CAN MIT 模式（底部关节） |
| 86 步进电机 | 2 | 脉冲/方向，MOTOR86 驱动 |
| 直流编码器电机 | 4 | BTS7960 H桥 + 编码器反馈 |
| STM32F103ZET6 | 1 | USART 串口接收 ROS2 命令 |

---

## 包说明

### stm32_robot_bridge

STM32 串口↔ROS2 话题桥接包，包含：

- **`serial_bridge_node`**：订阅 `/motor_commands` 话题，通过串口下发控制帧到 STM32
- **`teleop_keyboard_node`**：键盘输入 → 发布 `/motor_commands` 话题

自定义消息：

| 消息 | 字段 | 说明 |
|---|---|---|
| `MotorCommand.msg` | motor_id, command, value | 电机 ID + 命令类型 + 数值 |
| `MotorStatus.msg` | motor_id, position, velocity | 电机反馈状态 |

### stm32_serial_bridge

DM4310 参数调试 GUI 包（基于 tkinter + ROS2 参数服务），包含：

- **`tuning_slider_gui`**：实时滑条调节 PD 参数，通过 `SetParameters` 服务发送到固件节点

---

## 环境要求

- Ubuntu 22.04
- ROS2 Humble
- 依赖包：`rclcpp`, `rcl_interfaces`, `std_msgs`

---

## 构建

```bash
cd ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
```

单独构建某包：

```bash
colcon build --packages-select stm32_robot_bridge
```

---

## 运行

### 方式一：launch 文件（推荐）

```bash
source install/setup.bash
ros2 launch stm32_robot_bridge robot_bridge.launch.py
```

launch 文件同时启动 `serial_bridge_node` 和 `teleop_keyboard_node`，串口默认 `/dev/ttyUSB0:115200`。

### 方式二：分别启动节点

```bash
# 终端 1：串口桥接（连接 STM32）
ros2 run stm32_robot_bridge serial_bridge_node \
  --ros-args -p port_name:=/dev/ttyUSB0 -p baud_rate:=115200

# 终端 2：键盘控制
ros2 run stm32_robot_bridge teleop_keyboard_node
```

键盘操作：

| 按键 | 功能 |
|---|---|
| `W` / `S` | 加速 / 减速 |
| `A` / `D` | 反转 / 正转 |
| `1`–`9` | 速度档位 10%–90% |
| `Space` | 急停 |
| `Ctrl+C` | 退出 |

### 方式三：DM4310 参数调节 GUI

```bash
source install/setup.bash
ros2 launch stm32_serial_bridge stm32_tuning_gui.launch.py
```

---

## 话题 / 参数一览

```
/motor_commands   ← teleop_keyboard_node 发布，serial_bridge_node 订阅
/motor_status     ← serial_bridge_node 发布（STM32 反馈）
```

---

## 精简版（simplified 分支）

切换到 `simplified` 分支可查看降级后的 ROS2 工作空间，仅保留：
- `wheeltec_bridge`：WHEELTEC C10B 底盘控制
- `stm32_robot_bridge`：底盘状态 UDP→ROS 桥接
- `chassis_control`：CAN 差速驱动（仅 DM4310 + DM3510）
