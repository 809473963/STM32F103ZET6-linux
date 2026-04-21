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

## 树莓派指令

> PORT=$(ls -1 /dev/ttyUSB* /dev/ttyACM* 2>/dev/null | sort -V | tail -n 1)
> echo "Use serial port: $PORT"
> sudo socat -d -d TCP-LISTEN:8888,reuseaddr,fork FILE:$PORT,raw,echo=0,b115200
> Use serial port: /dev/ttyUSB0

## 已验证可用命令（新板）

以下命令在更换新 STM32 板后已验证可用：

```bash
cd /home/luo/stm-linux
sudo ./flash.sh encoder-serial-flymcu
```

## 通过树莓派远程烧录

如果 STM32 接在树莓派上，可在当前机器直接调用远程烧录脚本：

1. 先构建固件：

```bash
cd /home/luo/stm-linux
./flash.sh encoder-build
```

2. 通过树莓派烧录（脚本会先上传固件再在树莓派执行 stm32flash）：

```bash
cd /home/luo/stm-linux
./rpi_flash_stm32.sh --fw /home/luo/stm-linux/build_encoder/STM32_KEY_PROJECT.bin --user ubuntu --host 192.168.10.2 --serial /dev/ttyUSB0 --baud 115200
```

说明：

- 树莓派端需要可用 sudo 权限。
- 脚本已内置 FlyMcu 时序重试和手动 BOOT0 回退路径。
- 如果树莓派上的串口是 /dev/ttyUSB1 或 /dev/ttyACM0，请替换 --serial 参数。

如果需要先构建再烧录：

```bash
cd /home/luo/stm-linux
./flash.sh encoder-build
sudo ./flash.sh encoder-serial-flymcu
```

## 若依然无法握手

- 确认 BOOT0=1 后复位，再烧录。
- 确认串口接到 STM32 ROM Bootloader 支持的 UART（常见 USART1：PA9/PA10）。
- 排查占用进程：

```bash
PORT=$(ls -1 /dev/ttyUSB* /dev/ttyACM* 2>/dev/null | sort -V | tail -n 1)
fuser -v "$PORT"
```
