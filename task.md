# STM32 Keil → Linux 迁移任务

## 规划阶段
- [x] 分析现有 Keil 项目结构
- [x] 确认工具链和工具可用性
- [x] 编写实施计划并获取用户审核

## 构建系统
- [x] 创建 GCC 适用的链接脚本 ([STM32F103ZETx_FLASH.ld](file:///home/luo/%E5%AE%9E%E9%AA%8C2%20%E6%8C%89%E9%94%AE%E8%BE%93%E5%85%A5/STM32F103ZETx_FLASH.ld))
- [x] 适配 GCC 格式的启动文件 (`startup_stm32f103xe.s`)
- [x] 编写 CMakeLists.txt（支持自动发现源文件）
- [x] 创建 `arm-none-eabi-gcc` 工具链文件

## 一键编译 + 烧录
- [x] 编写 [flash.sh](file:///home/luo/%E5%AE%9E%E9%AA%8C2%20%E6%8C%89%E9%94%AE%E8%BE%93%E5%85%A5/flash.sh) 一键脚本（包含调用 STM32CubeProgrammer 烧录）
- [x] 编写 [setup.sh](file:///home/luo/%E5%AE%9E%E9%AA%8C2%20%E6%8C%89%E9%94%AE%E8%BE%93%E5%85%A5/setup.sh) 环境配置脚本（安装 gcc-arm-none-eabi 等）
- [x] 修复目录迁移后构建失败（`flash.sh` 自动检测并重建失效 CMakeCache）
- [x] 安装 STM32CubeProgrammer CLI（v2.22.0）并配置 PATH

## ROS2 集成 (USB 串口通信)
- [x] 创建 ROS2 功能包结构 (`stm32_robot_bridge`)
- [x] 实现 USB 串口桥接节点 (`serial_bridge_node`)
- [x] 实现键盘遥控多电机节点 (`teleop_keyboard_node`)
- [x] 定义自定义消息类型（电机控制、状态反馈）

## 机器人多电机控制架构
- [x] 设计通信协议（STM32 ↔ ROS2）
- [x] 创建电机驱动抽象层 (motor_driver)
- [x] 实现 PWM/定时器管理模块

## 验证
- [x] 验证 CMake 编译通过（`./flash.sh build`，2026-03-16）
- [ ] 验证烧录脚本（需硬件）
- [x] 验证 ROS2 节点可启动（在 ASCII 路径工作区验证通过）

> 已安装 `STM32_Programmer_CLI` 并可调用；当前烧录阻塞转为硬件连接问题：`No debug probe detected` / `Target device not found`。

> 说明：`rosidl_generate_interfaces` 在中文路径（如 `实验2_按键输入`）下会触发依赖解析异常（`Target dependency '2_' does not exist`）。ROS2 包请放在纯 ASCII 路径构建与运行。
