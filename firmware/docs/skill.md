# WSL STM32 串口自动挂载 Skill

## 目标

- WSL 启动时自动加载 CH340/CH341 驱动。
- STM32 插拔后自动给 /dev/ttyUSB* 和 /dev/ttyACM* 赋予可访问权限。

## 一次性安装

在 WSL 中执行：

```bash
cd /home/luo/stm-linux
sudo install -m 755 wsl_stm32_autoload.sh /usr/local/bin/wsl_stm32_autoload.sh
sudo cp /etc/wsl.conf /etc/wsl.conf.bak.$(date +%Y%m%d_%H%M%S) 2>/dev/null || true
printf '[boot]\ncommand=/usr/local/bin/wsl_stm32_autoload.sh\n' | sudo tee /etc/wsl.conf >/dev/null
sudo /usr/local/bin/wsl_stm32_autoload.sh
```

然后在 Windows PowerShell 执行：

```powershell
wsl --shutdown
```

重新打开 WSL 后生效。

## 日常验证

```bash
lsusb | grep -i 1a86
ls -l /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
```

## 烧录命令示例

```bash
PORT=$(ls -1 /dev/ttyUSB* /dev/ttyACM* 2>/dev/null | sort -V | tail -n 1)
/home/luo/STMicroelectronics/STM32Cube/STM32CubeProgrammer/bin/STM32_Programmer_CLI \
  -c port="$PORT" br=115200 P=EVEN \
  -w /home/luo/stm-linux/build_hello/STM32_KEY_PROJECT.bin 0x08000000 -v -rst
```

说明：WSL 中串口号可能在 /dev/ttyUSB0、/dev/ttyUSB1、/dev/ttyACM0 之间变化，不建议写死。

## 通过树莓派远程烧录 USER_ARM_DM 固件（当前主固件）

STM32 通过 CH9102 USB 桥接连接到树莓派 `/dev/ttyUSB0`，固件源码在 `USER_ARM_DM/main.c`。

### 第一步：构建固件

```bash
cd /home/luo/stm-linux
bash flash.sh build
```

输出 `build/STM32_KEY_PROJECT.bin`（约 20 KB）。

### 第二步：烧录到 STM32

脚本通过 SSH 把固件传到树莓派，再用 `stm32flash` 写入。CH9102 的 DTR/RTS 已连接 NRST/BOOT0，脚本会自动完成复位时序（FlyMcu 模式）：

```bash
cd /home/luo/stm-linux
bash rpi_flash_stm32.sh --fw build/STM32_KEY_PROJECT.bin
```

- 若 FlyMcu 自动时序失败，脚本会暂停并提示手动操作：把 BOOT0 拨到 1 → 断电重启 STM32 → 按 Enter。
- 烧录成功后 BOOT0 无需手动拨回，脚本的复位序列会自动切回应用模式。

### 验证固件运行

烧录完成后，在树莓派上被动监听 4 秒，应每秒收到一帧 `aa 11 83 ...` 状态心跳：

```bash
python3 - <<'PY'
import serial, time
ser = serial.Serial()
ser.port = '/dev/ttyUSB0'; ser.baudrate = 115200; ser.timeout = 0.2
ser.dsrdtr = False; ser.rtscts = False
ser.open(); ser.setDTR(False); ser.setRTS(False)
time.sleep(1.0); ser.flushInput()
t0 = time.time()
while time.time() - t0 < 4.0:
    b = ser.read(32)
    if b: print(b.hex(' '))
ser.close()
PY
```

> **重要**：打开 `/dev/ttyUSB0` 时必须加 `dsrdtr=False` 并调用 `setDTR(False)`。
> 该板 CH9102 的 DTR 直连 STM32 NRST，pyserial 默认 DTR=高会持续复位 STM32。

## 排查烧录失败

- **`Failed to init device, timeout`**：BOOT0 未拨到 1，或未断电复位。
- **`Failed to write memory`**：Flash 写入校验错，重试一次通常可解决。
- **串口被占用**：

```bash
sudo fuser -k /dev/ttyUSB0
```

## 树莓派 TCP 串口桥接

**不要用 socat**：socat 打开串口时不会设置 DTR=0 / RTS=0，导致 STM32 始终处于复位（DTR）或引导加载程序（RTS/BOOT0=1）状态，应用无法通信。

使用项目内的 Python 桥接脚本代替：

```bash
# 在树莓派上执行，传到 Pi 一次即可
# scp /home/luo/stm-linux/pi_serial_tcp_bridge.py ubuntu@192.168.10.2:~/

# 启动桥接（自动设 DTR=0 RTS=0，STM32 进入应用模式）
python3 ~/pi_serial_tcp_bridge.py
```

然后在 WSL 上运行键盘控制：

```bash
python3 pc_keyboard_control.py --socket-host 192.168.10.2
```

## pc_keyboard_control.py 控制模式完整速查

`pc_keyboard_control.py` 运行在 WSL 上，通过 TCP socket 连接树莓派串口桥（或直接在树莓派上运行）。

### 连接方式

```bash
# 方式 A：WSL 通过 TCP 连接（树莓派需先启动 socat）
cd /home/luo/stm-linux
python3 pc_keyboard_control.py --socket-host 192.168.10.2

# 方式 B：直接在树莓派上运行（SSH 进去后）
python3 pc_keyboard_control.py --port /dev/ttyUSB0

# 通信链路测试（验证后退出）
python3 pc_keyboard_control.py --socket-host 192.168.10.2 --comm-test
```

### 常用启动选项

| 选项 | 默认值 | 说明 |
|------|--------|------|
| `--motor 1` | 1 | 初始选中电机 ID（1=DM4310，2=DMH3510，255=全部） |
| `--mode vel` | vel | 控制模式：`vel`（rad/s）或 `duty`（占空比%） |
| `--speed-rad-s 0.4` | 0.40 | vel 模式下 A/D 的目标速度 |
| `--step 10` | 5 | duty 模式下 A/D 的目标占空比 % |
| `--startup-speed-rad-s 0.15` | 0.15 | vel 模式启动坡底速度 |
| `--startup-duty 5` | 5.0 | duty 模式启动坡底占空比 |
| `--ramp-ms 1200` | 1200 | 启动爬升时间（ms），0 禁用 |
| `--no-log` | — | 禁用 CSV 反馈日志 |

### 键盘按键说明

**运动控制**

| 按键 | 动作 |
|------|------|
| `D` | 正向运动（持续按住保持速度） |
| `A` | 反向运动 |
| `W` | 提高速度（vel +0.1 rad/s）或占空比（+2%） |
| `S` | 降低速度或占空比 |
| `Space` | 平滑减速停止并失能 |
| `Q` | 平滑停止并退出 |

**电机选择**

| 按键 | 动作 |
|------|------|
| `1` / `2` | 选中对应 ID 的电机（DM4310 / DMH3510） |
| `0` | 选中全部电机（ID=255） |
| `F` | 切换当前电机使能/失能 |
| `R` | 立即请求一次状态帧 |

**在线参数调节（运行中实时生效）**

| 按键 | 动作 |
|------|------|
| `[` / `]` | KP -0.5 / +0.5 |
| `;` / `'` | KD -0.05 / +0.05 |
| `,` / `.` | 重力补偿 KG -0.02 / +0.02 |
| `N` / `M` | 重力补偿偏置 bias -0.05 / +0.05 rad |
| `G` | 切换重力补偿开/关 |
| `B` | 以当前位置重置重力零点 |

**WHEELTEC 底盘控制**（需 `--chassis-port` 指定底盘串口）

| 按键 | 动作 |
|------|------|
| `I` | 底盘前进 |
| `K` | 底盘后退 |
| `J` | 底盘左转 |
| `L` | 底盘右转 |
| `U` | 底盘停止 |

> 底盘速度通过 `--chassis-vx`（前后 mm/s）和 `--chassis-vz`（转向 mrad/s）调节。

**KG 重力补偿调参流程**

| 步骤 | 按键 | 说明 |
|------|------|------|
| 1 | `F` | 使能电机 |
| 2 | `H` | 进入浮动模式（KP=0 KD=0，仅靠重力补偿） |
| 3 | `,` / `.` | 调节 KG 直到机械臂在任意角度悬停 |
| 4 | `E` | 锁存当前位置 |
| 5 | `]` | 逐步增加 KP 直到不漂移 |
| 6 | `C` | 移到水平位置后按 C 校准重力零点 |

**CAN 总线工具**

| 按键/命令行 | 动作 |
|-------------|------|
| `X` | 扫描 CAN 总线上的 DM 电机 ID |
| `--scan-ids` | 启动时扫描 ID 然后退出 |
| `--set-id 1 3` | 将 CAN ID 1 改为 3 然后退出 |

### 机械臂（2 电机）典型启动命令

```bash
# 仅机械臂，vel 模式，选中 DM4310 底部关节
python3 pc_keyboard_control.py --socket-host 192.168.10.2 --motor 1 --mode vel --speed-rad-s 0.4

# 低速安全测试
python3 pc_keyboard_control.py --socket-host 192.168.10.2 --motor 1 --mode vel --speed-rad-s 0.2 --no-log

# 全部电机 + 底盘控制
python3 pc_keyboard_control.py --socket-host 192.168.10.2 --motor 255 --mode vel \
  --chassis-port /dev/ttyACM0 --chassis-vx 300 --chassis-vz 500

# 仅底盘（不操作机械臂电机）
python3 pc_keyboard_control.py --socket-host 192.168.10.2 --chassis-port /dev/ttyACM0
```

### 底盘控制专用选项

| 选项 | 默认值 | 说明 |
|------|--------|------|
| `--chassis-port` | （空=禁用） | WHEELTEC 底盘串口，如 `/dev/ttyACM0` |
| `--chassis-baud` | 115200 | 底盘串口波特率 |
| `--chassis-vx` | 200 | 前进/后退速度 mm/s |
| `--chassis-vz` | 400 | 左右转向速度 mrad/s |

**电机 ID 与关节对应关系（简化后）：**

| ID | 电机型号 | 关节 | 说明 |
|----|----------|------|------|
| 1 | DM4310 | base_pitch | 机械臂底部关节，10:1 减速，峰值 7.0 N.m |
| 2 | DMH3510 | first_joint | 第一关节，直驱，峰值 0.45 N.m |

### 编码器电机（encoder 固件）

```bash
python3 pc_keyboard_control.py --port /dev/ttyUSB0 --mode duty --step 10 --startup-duty 6 --ramp-ms 900
```

### 86 步进电机（motor86 固件）

```bash
python3 pc_keyboard_control.py --port /dev/ttyUSB0 --mode vel --speed-rad-s 6.0 --startup-speed-rad-s 1.0 --ramp-ms 900
```

## 2025-04 重构变更记录

机械臂简化并耦合进 WHEELTEC 底盘小车，新增撑脚。

### 电机 ID 映射（4 电机）

| ID | 型号 | 用途 | 备注 |
|----|------|------|------|
| 1 | DM4310 | 机械臂底部关节 | 10:1 减速，峰值 7 N.m |
| 2 | DMH3510 | 机械臂第一关节 | 直驱，峰值 0.45 N.m |
| 3 | DMH3510 | 撑脚左 | 背对安装，与 ID4 同轴反向 |
| 4 | DMH3510 | 撑脚右 | 背对安装，与 ID3 同轴反向 |

### 固件改动

- `HARDWARE/PROTOCOL/protocol.c`：`ARM_MOTOR_COUNT` = 4（ID1~4），移除 Gripper42，新增撑脚电机配置
- `HARDWARE/PROTOCOL/protocol.h`：移除 `CMD_GRIPPER_*` 宏
- `CMakeLists.txt`：排除 `HARDWARE/GRIPPER42/` 源文件

### 统一控制程序 arm_car_control.py

**在树莓派上运行，一个程序控制底盘+机械臂+撑脚+摄像头。**

```bash
# 基本启动
python3 ~/stm-linux/arm_car_control.py

# 指定端口
python3 ~/stm-linux/arm_car_control.py --arm-port /dev/ttyUSB0 --car-port /dev/ttyACM0

# 带摄像头
python3 ~/stm-linux/arm_car_control.py --enable-cam --cam-devices 0 --enable-csi --csi-color rgb

# 仅底盘
python3 ~/stm-linux/arm_car_control.py --no-arm

# 翻转撑脚方向（如果装反了）
python3 ~/stm-linux/arm_car_control.py --leg-flip
```

按键映射：

| 区域 | 按键 | 功能 |
|------|------|------|
| 底盘 | `I/K/J/L` | 前进/后退/左转/右转（长按保持，松开停车） |
| 底盘 | `U/O` | 速度档+/- |
| 机械臂 | `A/D` | 电机反转/正转（长按保持） |
| 机械臂 | `W/S` | 臂速度+/-（步进 0.1 rad/s） |
| 机械臂 | `1/2` | 选电机 DM4310/DMH3510 |
| 机械臂 | `0` | 选全部 |
| 机械臂 | `F` | 使能/失能 |
| 撑脚 | `T` | 伸出（抬车）长按保持 |
| 撑脚 | `V` | 收回（降车）长按保持 |
| 调参 | `[/]` `;/'` `,/.` `N/M` | KP / KD / KG / bias |
| 调参 | `G` `B` `C` `H` `E` `Y` | 重力开关/零点/校准/浮动/锁存/复位bias |
| 共用 | `Space` | 全停（底盘+机械臂+撑脚） |
| 共用 | `P` | 打印底盘反馈 |
| 共用 | `R` | 请求 STM32 状态 |
| 共用 | `X` | 扫描 CAN 电机 ID |
| 共用 | `Q` | 退出 |

### 旧脚本 pc_keyboard_control.py

仍可单独使用（仅臂，2 电机），用于调试：
```bash
python3 pc_keyboard_control.py --port /dev/ttyUSB0 --motor 1
```

### 固件通信看门狗

`protocol.c` 新增 `ARM_COMM_WATCHDOG_MS = 500`：
- 每收到一帧合法协议帧，看门狗计数器清零
- 若 500ms 内没有任何合法帧 → `arm_stop_all()` 自动失能所有电机
- Python 端 `--status-period-ms 200`（默认值）提供周期心跳，正常运行不会触发看门狗
- 保护场景：SSH 断连、串口线脱落、Python 崩溃

### 安全链路总结

| 故障场景 | 底盘行为 | 机械臂行为 |
|---------|---------|-----------|
| SSH 断开 | 心跳停 → 自动停车 ✅ | 看门狗 500ms → 自动失能 ✅ |
| 串口线脱落 | N/A（有线） | 看门狗 500ms → 自动失能 ✅ |
| Python 崩溃 | 心跳停 → 自动停车 ✅ | 看门狗 500ms → 自动失能 ✅ |
| 正常退出(Q) | Python 发停止帧 ✅ | Python 发失能帧 ✅ |

### 运行架构

```
WSL/PC ──(网线)──> 树莓派 (192.168.10.2)
                    ├── /dev/ttyUSB0 ──> STM32 (机械臂 DM4310+DMH3510)
                    ├── /dev/ttyACM0 ──> WHEELTEC (底盘)
                    └── 摄像头 → Flask web → http://192.168.10.2:8080
```

- WSL 用途：编译固件 (`bash flash.sh build`) + 烧录 (`bash rpi_flash_stm32.sh`)
- 日常控制：SSH 进树莓派，直接运行控制程序（无需 TCP 桥）
- 树莓派可直接通过 `/dev/ttyUSB0` 控制 STM32，无需中间层

### 注意事项

- 原文件备份为 `pc_keyboard_control.py.bak`
- 底盘控制需通过 `--chassis-port` 显式启用，否则 I/J/K/L 按键不产生效果
- 底盘协议：`[0x7B] [flag] [vx_H vx_L] [vy_H vy_L] [vz_H vz_L] [BCC] [0x7D]`，int16 大端
- 固件烧录后需重新上电或复位才能激活看门狗
