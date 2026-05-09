# ARM DM Control Guide

本指南对应 `ARM_DM_APP` 固件（`USER_ARM_DM/main.c`）与 `pc_keyboard_control.py`。

电机约定：

- DM4310：`id=1`（底座关节）
- DM3510：`id=2/3/4`（机械臂关节）
- YOBOTICS M24：目标 `id=5`（绕 z 轴）

## 1. 固件构建与烧录

```bash
cd /home/luo/stm-linux
./flash.sh arm-dm-build
sudo ./flash.sh arm-dm-serial-flymcu /dev/ttyUSB0 115200
```

## 2. M24 改 ID 为 5

先扫描，再改 ID（old_id 按扫描结果填写）：

```bash
cd /home/luo/stm-linux
python3 pc_keyboard_control.py --port /dev/ttyUSB0 --baud 115200 --scan-ids
python3 pc_keyboard_control.py --port /dev/ttyUSB0 --baud 115200 --set-id <old_id> 5
python3 pc_keyboard_control.py --port /dev/ttyUSB0 --baud 115200 --scan-ids
```

## 3. 单个电机调试（重点）

单电机调试主程序就在：

- `stm-linux/pc_keyboard_control.py`（键盘发命令 + 实时读反馈）

### 3.1 先启动单电机调试

以底座 `DM4310(id=1)` 为例：

```bash
cd /home/luo/stm-linux
python3 pc_keyboard_control.py --port /dev/ttyUSB0 --baud 115200 --mode vel --motor 1 --status-period-ms 200
```

其他电机直接改 `--motor`：

```bash
# DM3510
python3 pc_keyboard_control.py --port /dev/ttyUSB0 --baud 115200 --mode vel --motor 2 --status-period-ms 200
python3 pc_keyboard_control.py --port /dev/ttyUSB0 --baud 115200 --mode vel --motor 3 --status-period-ms 200
python3 pc_keyboard_control.py --port /dev/ttyUSB0 --baud 115200 --mode vel --motor 4 --status-period-ms 200

# M24
python3 pc_keyboard_control.py --port /dev/ttyUSB0 --baud 115200 --mode vel --motor 5 --status-period-ms 200
```

### 3.2 调试时按键

按键：

- `A/D`：反向/正向
- `W/S`：增/减速度（或占空比）
- `Space`：停机
- `1~5`：运行中切换当前调试电机
- `0`：切换为全电机广播（id=255）
- `R`：立即请求一次反馈（推荐频繁按）
- `X`：扫描总线 ID
- `Q`：退出并停机（安全退出）

### 3.3 看哪里是“负载反馈”

终端里会打印反馈行：

- `[FB] id=2 en=1 pos=... rad vel=... rad/s tau=... Nm st=...`

重点看：

- `tau`：当前负载/力矩估计（调重力补偿最有用）
- `vel`：速度跟随情况（调 `kp/kd` 关键）
- `st`：状态码（非 0 要先排故）

## 4. 多电机联动控制

单电机调好后，再切到全电机广播：

1. 启动任一单电机命令进入控制界面；
2. 按 `0` 切到 `ALL(id=255)`；
3. 用 `A/D/W/S` 做整臂联动验证。

## 5. 单电机调参与重力补偿建议

建议先逐个电机调试，再联动：

1. 固定 `kp/kd` 低值起步，先保证无明显自激振荡。
2. 以小幅速度命令往复（`A/D`）观察 `tau` 与 `vel`：
   - 跟随慢：适度增大 `kp`
   - 震荡大：降低 `kp` 或增大 `kd`
   - 静态保持吃力：增加 `t_ff`（重力补偿项）
3. 对不同姿态分别记录保持所需 `t_ff`，拟合为角度函数（可先用分段线性）。
4. 再把该补偿写入 ROS2 控制节点，在线叠加到力矩前馈。

`ros2_ws/src/stm32_serial_bridge/stm32_serial_bridge/tuning_slider_gui.py` 已可可视化调参并显示状态反馈，建议作为单电机调参主入口。

## 6. ROS 控制整臂（结合 arm_ws URDF）

你当前 `arm_ws` 下已有 URDF/MoveIt 配置（`my_4dof_arm_moveit_config`）。推荐链路：

1. MoveIt/Servo 输出关节目标（位置/速度）。
2. ROS2 桥接节点按关节映射到电机 `id=1..5`。
3. 桥接节点将每关节目标转成 STM32 `CMD_GATEWAY_CONTROL`（MIT 参数）。
4. STM32 回传 `0x83` 状态后发布为 ROS 反馈话题，供 RViz/调参界面闭环观测。

最小映射建议：

- `joint_base` -> `id=1` (DM4310)
- `joint_1/2/3` -> `id=2/3/4` (DM3510)
- `joint_z` -> `id=5` (M24)

上线前检查：

- 关节方向与 URDF 正方向一致（必要时在桥接层加符号翻转）。
- 每个关节的速度/力矩限幅与电机物理上限一致。
- 上电默认 `enable=0`，只有收到有效目标后再使能，丢包超时自动急停。

### 6.1 首次安装 RViz/MoveIt 依赖（Humble）

```bash
sudo apt-get update
sudo apt-get install -y ros-humble-rviz2
sudo apt-get install -y ros-humble-moveit-configs-utils ros-humble-moveit
sudo apt-get install -y ros-humble-controller-manager ros-humble-ros2-control ros-humble-ros2-controllers
```

### 6.2 启动图形化拖动（RViz + MoveIt）

```bash
cd /home/luo/arm_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch my_4dof_arm_moveit_config demo.launch.py
```

若仅验证 RViz 可用：

```bash
source /opt/ros/humble/setup.bash
rviz2
```

## 7. 42 步进夹爪（TB6600，无传感器零点）

夹爪单独采用“人工设零 + 软限位”方案，详细流程见：

- `GRIPPER_42_TB6600_ZEROING.md`

建议先严格执行该文档里的首次标定和上电确认，再接入整臂联动。

快速命令：

```bash
cd /home/luo/stm-linux
python3 pc_keyboard_control.py --port /dev/ttyUSB0 --baud 115200 --gripper-trust-zero
python3 pc_keyboard_control.py --port /dev/ttyUSB0 --baud 115200 --gripper-set-zero
python3 pc_keyboard_control.py --port /dev/ttyUSB0 --baud 115200 --gripper-save-cfg
python3 pc_keyboard_control.py --port /dev/ttyUSB0 --baud 115200 --gripper-open-pct 40
```
