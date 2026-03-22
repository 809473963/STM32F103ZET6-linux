#!/bin/bash
# setup.sh - Environment setup script for STM32 development

set -e

echo "=== STM32 Linux Development Environment Setup ==="
echo "This script will install the GCC ARM embedded toolchain and necessary build tools."

# Check for root/sudo
if [ "$EUID" -ne 0 ] && [ -x "$(command -v sudo)" ]; then
    SUDO="sudo"
else
    SUDO=""
fi

echo "Updating package lists..."
$SUDO apt-get update

echo "Installing build tools and ARM GCC..."
$SUDO apt-get install -y gcc-arm-none-eabi binutils-arm-none-eabi gdb-multiarch \
                         cmake ninja-build make nano unzip libnewlib-arm-none-eabi

echo ""
echo "=== STM32CubeProgrammer Required ==="
echo "Since you chose STM32CubeProgrammer for flushing, you need to install it manually."
echo "1. Download it from ST's official website: https://www.st.com/en/development-tools/stm32cubeprog.html"
echo "2. Put the installer package into this project directory and run: ./install_cubeprogrammer.sh"
echo "3. The script installs CubeProgrammer and appends its bin path to ~/.bashrc automatically."

echo ""
echo "=== ROS2 Setup ==="
if [ -d "/opt/ros/humble" ]; then
    echo "ROS2 Humble detected successfully."
else
    echo "WARNING: ROS2 Humble does not seem to be sourced or installed in /opt/ros/humble."
fi

# Make our helper scripts executable
chmod +x flash.sh
chmod +x install_cubeprogrammer.sh

echo "Setup complete!"
echo "Run './flash.sh' to compile and flash the firmware."
