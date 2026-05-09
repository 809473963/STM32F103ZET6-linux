#!/bin/bash
# ============================================================
#  setup_rpi_network.sh
#  在树莓派上一键配置：
#   1. 网线口 (eth0) 静态 IP 192.168.10.2
#   2. 授予串口权限 (dialout 组)
#   3. 写入 ROS2 环境变量到 ~/.bashrc
#
#  使用: bash setup_rpi_network.sh
# ============================================================
set -e

RPI_ETH_IP="192.168.10.2"
RPI_ETH_GW="192.168.10.1"
PC_IP="192.168.10.1"
DOMAIN_ID=10

echo "========================================="
echo "  WHEELTEC 树莓派网络 & ROS2 配置脚本"
echo "========================================="

# ── 1. 静态 IP（通过 NetworkManager / nmcli）────────────────
if command -v nmcli &>/dev/null; then
    CONN=$(nmcli -t -f NAME,DEVICE con show --active | grep eth0 | cut -d: -f1)
    if [ -z "$CONN" ]; then
        CONN="wired-wheeltec"
        nmcli con add type ethernet ifname eth0 con-name "$CONN"
    fi
    nmcli con modify "$CONN" \
        ipv4.method manual \
        ipv4.addresses "${RPI_ETH_IP}/24" \
        ipv4.gateway  "$RPI_ETH_GW" \
        ipv4.dns      "8.8.8.8"
    nmcli con up "$CONN"
    echo "[OK] eth0 静态 IP → ${RPI_ETH_IP}/24"
else
    # 备用: dhcpcd / netplan
    if [ -f /etc/dhcpcd.conf ]; then
        grep -q "interface eth0" /etc/dhcpcd.conf || cat >> /etc/dhcpcd.conf <<EOF

interface eth0
static ip_address=${RPI_ETH_IP}/24
static routers=${RPI_ETH_GW}
static domain_name_servers=8.8.8.8
EOF
        echo "[OK] dhcpcd.conf 已写入静态 IP ${RPI_ETH_IP}"
    else
        echo "[WARN] 未找到 NetworkManager 或 dhcpcd，请手动配置 IP"
    fi
fi

# ── 2. 串口权限 ────────────────────────────────────────────
sudo usermod -aG dialout "$USER"
echo "[OK] 已将 $USER 加入 dialout 组（重新登录后生效）"
echo "     立即生效可运行: newgrp dialout"

# ── 3. ROS2 环境变量写入 ~/.bashrc ─────────────────────────
BASHRC="$HOME/.bashrc"
MARKER="# ── WHEELTEC ROS2 ENV ──"
if ! grep -q "$MARKER" "$BASHRC"; then
    cat >> "$BASHRC" <<EOF

$MARKER
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=${DOMAIN_ID}
# 跨网段多播时开启下面两行（单网段 Ethernet 通常不需要）
# export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
# export FASTRTPS_DEFAULT_PROFILES_FILE=\$HOME/fastdds_rpi.xml
EOF
    echo "[OK] ROS2 环境变量已写入 ~/.bashrc (ROS_DOMAIN_ID=${DOMAIN_ID})"
else
    echo "[SKIP] ~/.bashrc 中 ROS2 环境变量已存在"
fi

# ── 4. 提示 PC 端要做什么 ─────────────────────────────────
cat <<MSG

=========================================
  树莓派配置完成！
=========================================
PC 端（Windows/Ubuntu）需要做：

  [Ubuntu PC]
    在 ~/.bashrc 追加:
      export ROS_DOMAIN_ID=${DOMAIN_ID}
    网线口设静态 IP: ${PC_IP}/24
    测试: ping ${RPI_ETH_IP}

  [Windows PC]
    网线适配器 → IPv4 → 手动
      IP:      ${PC_IP}
      子网掩码: 255.255.255.0
    然后在 WSL2 Ubuntu 里:
      export ROS_DOMAIN_ID=${DOMAIN_ID}

验证通信:
  Pi:  ros2 topic pub /test std_msgs/msg/String '{data: "hello"}' -r 1
  PC:  ros2 topic echo /test

MSG
