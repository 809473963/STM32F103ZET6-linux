# WHEELTEC 履带小车 — 电脑网线→树莓派→有线→小车 完整指南

## 1. 硬件连接

```
PC（键盘）
  │  网线（RJ45）
  ▼
Raspberry Pi（ROS2 Humble）
  │  USB-A → USB-Micro/Mini-B 线（普通数据线）
  ▼
WHEELTEC C10B 主板 USB 口（CH9102 USB-Serial）
  │  内部连接
  ▼
STM32 UART1
  │
左/右电机驱动 → 履带底盘
```

### 为什么用 USB 口？
- C10B 主板自带 **CH9102 USB-Serial 芯片**，USB 口即串口，无需额外转换器
- 协议与蓝牙/WiFi **完全相同**（厂家手册已确认），零固件修改
- 树莓派端可能识别为 `/dev/ttyACM0` 或 `/dev/ttyUSB0`，波特率 115200

---

## 2. 需要修改 / 新增的内容

| 项目 | 状态 | 说明 |
|------|------|------|
| 蓝牙/WiFi 模块 | 无需改动 | 留着备用，不冲突 |
| STM32 固件 | **无需改动** | 使用现有串口1协议 |
| 树莓派网络 | **需配置** | eth0 静态 IP 192.168.10.2 |
| PC 网络 | **需配置** | 有线网卡静态 IP 192.168.10.1 |
| `wheeltec_bridge` 包 | **已创建** | 串口节点 + 键盘节点 |

---

## 3. 树莓派一键配置

在树莓派上执行（只需做一次）：

```bash
# 将 wheeltec_bridge 目录拷贝到树莓派（或 git clone 整个 aia_ws）
scp -r ~/aia_ws/src/wheeltec_bridge ubuntu@192.168.10.2:~/

# SSH 登录树莓派
ssh ubuntu@192.168.10.2

# 执行配置脚本
bash ~/wheeltec_bridge/setup_rpi_network.sh

# 安装系统依赖（推荐 apt，避免 PEP668 报错）
sudo apt update
sudo apt install -y python3-serial python3-flask python3-opencv python3-picamera2

# 构建 ROS2 包
mkdir -p ~/aia_ws/src
mv ~/wheeltec_bridge ~/aia_ws/src/
cd ~/aia_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select wheeltec_bridge
source install/setup.bash
```

---

## 4. PC 端配置

**Windows（通过 WSL2）**：
1. 有线网卡 → IPv4 → 手动：IP `192.168.10.1`，子网 `255.255.255.0`
2. WSL2 终端（每次打开都要）：
   ```bash
   export ROS_DOMAIN_ID=10
   source /opt/ros/humble/setup.bash
   ```

**Ubuntu PC**：
```bash
# 设置有线网卡静态 IP（图形界面或 nmcli）
sudo nmcli con modify "有线连接" ipv4.method manual \
    ipv4.addresses 192.168.10.1/24

# 写入 ~/.bashrc（永久生效）
echo "export ROS_DOMAIN_ID=10" >> ~/.bashrc
source ~/.bashrc
```

---

## 5. 启动步骤

### 方式一：键盘控制在树莓派本地（推荐，延迟最低）

**树莓派 — 终端1**（启动串口节点）：
```bash
cd ~/aia_ws && source install/setup.bash
ros2 launch wheeltec_bridge wheeltec_teleop.launch.py port:=/dev/ttyACM0
```

**树莓派 — 终端2**（SSH 另一窗口，启动键盘）：
```bash
cd ~/aia_ws && source install/setup.bash
ros2 run wheeltec_bridge wheeltec_keyboard
```

### 方式二：键盘控制在 PC（通过 ROS2 网络）

**树莓派**（只启动串口节点）：
```bash
cd ~/aia_ws && source install/setup.bash
ros2 launch wheeltec_bridge wheeltec_teleop.launch.py
```

**PC**（键盘控制，需安装 teleop_twist_keyboard）：
```bash
# 安装（只需一次）
sudo apt install ros-humble-teleop-twist-keyboard

# 启动
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=10
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
> 线速度阈值 0.05 m/s，角速度阈值 0.1 rad/s，
> `teleop_twist_keyboard` 默认值均高于此阈值，可直接使用

### 方式三：用自带键盘节点（更精简）
PC 通过 SSH 直接在树莓派上运行键盘节点（无需 PC 装 ROS2）：
```bash
ssh -t ubuntu@192.168.10.2 "cd ~/aia_ws && source install/setup.bash && ros2 run wheeltec_bridge wheeltec_keyboard"
```

---

## 6. 验证连接

```bash
# 树莓派：查看串口设备
ls -l /dev/ttyACM* /dev/ttyUSB*

# 查看 ROS2 话题
ros2 topic list         # 应看到 /cmd_vel, /wheeltec/speed_set

# PC：验证跨机通信
ros2 topic echo /cmd_vel
```

---

## 7. 是否加入 ROS？ ——推荐加入，理由如下

| 功能 | 不加 ROS | 加 ROS |
|------|---------|--------|
| PC 键盘控制 | ✓ SSH 脚本 | ✓ teleop_twist_keyboard |
| 多机协作 | ✗ 困难 | ✓ DDS 自动发现 |
| 雷达/导航 | ✗ 需手写桥接 | ✓ nav2 直接接入 |
| 与机械臂联动 | ✗ 复杂 | ✓ 同一 ROS 网络 |
| 录制回放调试 | ✗ | ✓ ros2 bag |

目前已创建的 `wheeltec_bridge` 已经是 ROS2 包，后续扩展：
- 解析 STM32 串口回传数据 → 发布 `/odom`
- 接入 LD14P 雷达 → `/scan`
- 使用 `nav2` 自主导航

---

## 8. 进阶：使用 UART4 ROS 接口（官方方案，需更新固件）

厂家 2025-05-15 更新的固件已支持 **UART4 作为 ROS 接口**，可发送连续速度值（而非离散方向字节），实现更精确的速度控制。

连接方式：树莓派 USB-Serial 转换器 → C10B 主板 UART4 针脚

详见：`5.STM32源码/` 中最新固件的串口4配置部分。

---

## 9. 按键速查（`wheeltec_keyboard` 节点）

```
  u  i  o    左前  前  右前
  j  k  l    左    停  右
  m  ,  .    左后  后  右后
  w = 加速    e = 减速    q = 退出
```

---

## 10. 一体化方案（推荐）：键盘 + 小车 + 双摄网页

当前推荐直接使用一体化脚本：`~/wheeltec_all_in_one.py`

功能：
- 键盘长按控制小车（松开自动停车）
- 浏览器同时显示 USB + CSI 两路摄像头
- 右侧状态区实时显示电压/速度/IMU（无整页刷新，不闪烁）

### 启动命令

```bash
python3 ~/wheeltec_all_in_one.py --cam-devices 0 --enable-csi
```

默认说明：
- CSI 颜色默认 `rgb`（已修复蓝色偏色问题）
- 网页地址：`http://192.168.10.2:8080`

### 常用参数

```bash
# 手动指定 CSI 颜色模式（默认 rgb）
python3 ~/wheeltec_all_in_one.py --cam-devices 0 --enable-csi --csi-color rgb

# 仅看摄像头，不连小车串口
python3 ~/wheeltec_all_in_one.py --cam-devices 0 --enable-csi --no-car

# 只控车，不启摄像头
python3 ~/wheeltec_all_in_one.py --no-cam
```

---

## 11. WSL 侧 RViz 看车（树莓派无 ROS）

如果你坚持“所有 ROS 都在 WSL”，树莓派只负责硬件（串口/摄像头），那么在 WSL 可直接查看履带车模型：

```bash
cd ~/aia_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch wheeltec_bridge view_chassis.launch.py
```

说明：
- 该 launch 会启动 `robot_state_publisher` + `rviz2`；
- 使用内置简化底盘模型：`wheeltec_bridge/urdf/wheeltec_chassis.urdf`；
- RViz 配置文件：`wheeltec_bridge/rviz/wheeltec_chassis.rviz`；
- 这是可视化模型，不依赖树莓派 ROS。
