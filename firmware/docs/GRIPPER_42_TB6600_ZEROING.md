# Gripper 42 Stepper (TB6600) No-Sensor Zeroing

本方案用于：42 步进电机 + TB6600 驱动夹爪，**无传感器**情况下通过“人工零点标定 + 软限位”避免过张/过合。

## 1. 核心原则

- 不做绝对位置假设：上电后默认“位置不可信”。
- 先人工标定零点，再允许自动开合。
- 全程限制在安全步数区间，禁止越界。

## 2. 坐标与限制定义

约定：

- `pos_steps`：当前步数位置（相对零点）
- `zero_steps = 0`：人工确认的“安全基准开口”
- `min_steps`：最小开口边界（防过合）
- `max_steps`：最大开口边界（防过张）
- `safe_margin_steps`：边界保护余量（建议 3%~8% 全行程）

运行允许区间：

- `[min_steps + safe_margin_steps, max_steps - safe_margin_steps]`

## 3. 首次标定流程（必须）

1. 上电后进入 `UNHOMED` 状态（仅允许点动）。
2. 用低速点动将夹爪调整到你认可的“标准开口”位置。
3. 执行“设零点”命令：`SET_ZERO`（将当前 `pos_steps` 置 0）。
4. 执行“保存标定”命令：`SAVE_ZERO_CFG`（保存到 Flash）。
5. 再做一次小范围开合，确认不触碰机械极限。

## 4. 每次上电流程（无传感器）

1. 读取 Flash 中上次保存的标定与安全参数。
2. 进入 `UNHOMED`，禁止自动任务直接驱动夹爪。
3. 操作者二选一：
   - 若确认机械未被外力移动，执行 `TRUST_LAST_ZERO`，进入 `READY`；
   - 若不确认，重复“首次标定流程”。

## 5. 何时强制重标定

以下任一条件触发后，必须重标定：

- 夹爪被手动掰动/撞击；
- 明显丢步（命令位移与实际开合不一致）；
- TB6600 过流/过热后恢复；
- 断电后机构被移动过。

## 6. 开合控制建议

- 输入不要直接给“无限转动”，统一用目标开度百分比：
  - `open_pct in [0,100]`
  - `target_steps = clamp(map(open_pct), min_steps + margin, max_steps - margin)`
- 靠近边界自动降速（例如剩余 15% 行程时线性降速）。
- 关合末端默认留余量，不打死。

## 7. 建议默认参数（起步）

- `jog_speed_steps_s`: 150~300
- `run_speed_steps_s`: 400~900
- `acc_steps_s2`: 600~1500
- `safe_margin_steps`: 总行程的 5%
- `max_single_move_steps`: 总行程的 25%（防误命令）

## 8. 与现有系统集成建议

- 将夹爪定义为逻辑轴 `gripper_id = 6`（不影响 DM4310/DM3510/M24）。
- 串口协议新增最小命令：
  - `CMD_GRIPPER_JOG`
  - `CMD_GRIPPER_SET_ZERO`
  - `CMD_GRIPPER_SAVE_CFG`
  - `CMD_GRIPPER_TRUST_LAST_ZERO`
  - `CMD_GRIPPER_SET_OPEN_PCT`
  - `CMD_GRIPPER_STATUS`
- 状态至少包含：
  - `state(UNHOMED/HOMING/READY/FAULT)`
  - `pos_steps`
  - `target_steps`
  - `min/max/margin`
  - `zero_valid`

## 10. 现已支持的上位机命令

`pc_keyboard_control.py` 已接入夹爪命令，可直接使用：

```bash
cd /home/luo/stm-linux

# 信任上次零点（上电后第一步）
python3 pc_keyboard_control.py --port /dev/ttyUSB0 --baud 115200 --gripper-trust-zero

# 手工点动到标准开口后，设零并保存
python3 pc_keyboard_control.py --port /dev/ttyUSB0 --baud 115200 --gripper-set-zero
python3 pc_keyboard_control.py --port /dev/ttyUSB0 --baud 115200 --gripper-save-cfg

# 直接设置开度百分比
python3 pc_keyboard_control.py --port /dev/ttyUSB0 --baud 115200 --gripper-open-pct 35
```

键盘在线调试（进入常规控制界面后）：

- `J/L`：夹爪关/开点动
- `K`：夹爪停止
- `O/P`：开度目标 -5% / +5%
- `U`：查询夹爪状态
- `Z`：设零点
- `V`：保存配置
- `T`：信任上次零点

## 9. 风险提示

无传感器方案本质上依赖“人工确认与纪律流程”，可靠性低于硬件回零。  
若后续允许改硬件，优先补一个低成本限位开关。
