# stm-linux

基于 Linux 的 STM32F103ZET6 开发工程，目标是将原有 Keil 工程迁移到 GCC + CMake 工作流，并提供一键构建、烧录与 ROS2 串口桥接能力。

## 1. 项目简介

本仓库主要解决三个问题：

1. 在 Linux 环境中稳定构建 STM32 固件（不依赖 Keil）。
2. 使用 STM32CubeProgrammer CLI 进行 SWD、DFU、UART 烧录。
3. 通过 ROS2 节点实现上位机键盘控制与串口桥接。

核心参数如下：

- MCU：STM32F103ZE（Cortex-M3）
- 编译工具链：arm-none-eabi-gcc
- 构建系统：CMake
- 烧录工具：STM32_Programmer_CLI
- 产物：`.elf`、`.hex`、`.bin`

## 2. 目录说明

- `CORE/`：内核相关代码与启动文件
- `HARDWARE/`：LED、按键、电机、协议等硬件驱动
- `SYSTEM/`：系统级公共组件（延时、串口等）
- `USER/`：应用层入口与 STM32 配置
- `STM32F10x_FWLib/`：标准外设库
- `ros2_ws/src/stm32_robot_bridge/`：ROS2 串口桥接与键盘控制包
- `CMakeLists.txt`：固件构建入口
- `arm-none-eabi-gcc.cmake`：交叉编译工具链配置
- `STM32F103ZETx_FLASH.ld`：链接脚本
- `flash.sh`：构建/烧录一体化脚本
- `setup.sh`：Linux 开发环境安装脚本
- `install_cubeprogrammer.sh`：CubeProgrammer 安装辅助脚本

## 3. 环境准备

### 3.1 安装基础依赖（Ubuntu/Debian）

```bash
sudo apt-get update
sudo apt-get install -y \
  gcc-arm-none-eabi \
  binutils-arm-none-eabi \
  gdb-multiarch \
  cmake ninja-build make unzip \
  libnewlib-arm-none-eabi
```

### 3.2 安装 STM32CubeProgrammer

从 ST 官网下载 Linux 安装包后，放到工程根目录执行：

```bash
./install_cubeprogrammer.sh
source ~/.bashrc
```

验证 CLI：

```bash
STM32_Programmer_CLI --version
```

### 3.3 首次初始化脚本

```bash
chmod +x setup.sh flash.sh install_cubeprogrammer.sh
./setup.sh
```

## 4. 固件构建

推荐使用封装脚本：

```bash
./flash.sh build
```

脚本行为：

- 自动创建 `build/`
- 自动检测并清理失效的 CMake 缓存（路径迁移后常见）
- 生成 `.elf`、`.hex`、`.bin`



若你需要手工执行 CMake：

```bash
cmake -S . -B build \
  -DCMAKE_TOOLCHAIN_FILE=arm-none-eabi-gcc.cmake \
  -DCMAKE_BUILD_TYPE=Release
cmake --build build -j"$(nproc)"
```

## 5. 烧录方法

### 5.1 一键构建并烧录

```bash
./flash.sh
```

### 5.2 仅烧录（默认 SWD）

```bash
./flash.sh flash
```

流程说明：

1. 优先尝试 SWD（ST-Link）。
2. SWD 失败后自动回退 USB1（DFU）模式。

### 5.3 UART 烧录（CubeProgrammer）

```bash
./flash.sh serial /dev/ttyUSB0 115200
```

注意：

- 进入系统 Bootloader 前通常需要将 BOOT0 置 1 并复位。
- 脚本默认串口参数包含偶校验（EVEN）。

### 5.4 UART 烧录（stm32flash，FlyMcu 时序）

```bash
sudo apt-get install -y stm32flash
./flash.sh serial-flymcu /dev/ttyUSB0 115200
```

## 6. ROS2 集成

ROS2 功能位于：`ros2_ws/src/stm32_robot_bridge/`，包含：

- `serial_bridge_node.cpp`：ROS2 话题与串口协议桥接
- `teleop_keyboard_node.cpp`：键盘控制节点
- `msg/`：电机命令与状态消息定义
- `launch/`：启动文件

构建示例：

```bash
cd ros2_ws
colcon build --packages-select stm32_robot_bridge
```

运行示例：

```bash
source install/setup.bash
ros2 run stm32_robot_bridge serial_bridge_node
ros2 run stm32_robot_bridge teleop_keyboard_node
```

## 7. 常见问题

### 7.1 `STM32_Programmer_CLI: command not found`

- 原因：CubeProgrammer 未安装或 PATH 未生效。
- 处理：重新执行 `./install_cubeprogrammer.sh`，并 `source ~/.bashrc`。

### 7.2 `No debug probe detected`

- 原因：ST-Link 未识别、接线不正确或目标板未上电。
- 处理：检查 USB、SWD 接线、供电与权限。

### 7.3 DFU 报 `Target device not found`

- 原因：未正确进入 DFU 模式。
- 处理：确认 BOOT 引脚设置与复位时序。

### 7.4 构建目录迁移后 CMake 报错

- 原因：`build/CMakeCache.txt` 记录了旧路径。
- 处理：`flash.sh` 已内置检测并自动重建构建目录。

### 7.5 ROS2 在中文路径下接口构建异常

- 现象：`rosidl_generate_interfaces` 相关路径解析错误。
- 建议：ROS2 工作区使用纯 ASCII 路径。

## 8. 清理与维护

清理固件构建目录：

```bash
./flash.sh clean
```

如需清理 ROS2 构建产物：

```bash
rm -rf ros2_ws/build ros2_ws/install ros2_ws/log
```

## 9. 后续可选改进

1. 增加 CI：自动执行交叉编译并上传产物。
2. 完善协议文档：补齐串口帧格式和命令表。
3. 增加硬件接线图：降低首次上手门槛。

## 10. 许可

当前仓库尚未明确 License，如需开源发布建议补充 `LICENSE` 文件。
