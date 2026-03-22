#!/bin/bash
# install_cubeprogrammer.sh - Install STM32CubeProgrammer from a local installer package

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SYSTEM_INSTALL_ROOT="/usr/local/STMicroelectronics/STM32Cube/STM32CubeProgrammer"
USER_INSTALL_ROOT="${HOME}/.local/STMicroelectronics/STM32Cube/STM32CubeProgrammer"
DEFAULT_RUN="${SCRIPT_DIR}/SetupSTM32CubeProgrammer*.linux"
DEFAULT_ZIP="${SCRIPT_DIR}/en.stm32cubeprg-lin*.zip"

print_usage() {
    echo "Usage: $0 [installer_file_or_glob]"
    echo ""
    echo "Examples:"
    echo "  $0"
    echo "  $0 ./en.stm32cubeprg-lin-v2.18.0.zip"
    echo "  $0 ./SetupSTM32CubeProgrammer-2.18.0.linux"
}

pick_installer() {
    local arg="${1:-}"
    if [ -n "$arg" ]; then
        echo "$arg"
        return
    fi

    local run_match zip_match
    run_match=$(compgen -G "$DEFAULT_RUN" | head -n 1 || true)
    zip_match=$(compgen -G "$DEFAULT_ZIP" | head -n 1 || true)

    if [ -n "$run_match" ]; then
        echo "$run_match"
        return
    fi
    if [ -n "$zip_match" ]; then
        echo "$zip_match"
        return
    fi

    echo ""
}

require_cmd() {
    if ! command -v "$1" >/dev/null 2>&1; then
        echo "Error: required command '$1' not found."
        exit 1
    fi
}

if [ "${1:-}" = "-h" ] || [ "${1:-}" = "--help" ]; then
    print_usage
    exit 0
fi

INSTALLER=$(pick_installer "${1:-}")
if [ -z "$INSTALLER" ]; then
    echo "Error: no STM32CubeProgrammer installer found in ${SCRIPT_DIR}."
    echo "Please download installer from ST website and place it here, then rerun."
    exit 1
fi

if [ ! -e "$INSTALLER" ]; then
    echo "Error: installer '$INSTALLER' does not exist."
    exit 1
fi

if [ -x "$(command -v sudo)" ] && [ "${EUID}" -ne 0 ]; then
    SUDO="sudo"
else
    SUDO=""
fi

INSTALL_ROOT="$SYSTEM_INSTALL_ROOT"
if [ "${EUID}" -ne 0 ] && [ ! -w "/usr/local" ]; then
    INSTALL_ROOT="$USER_INSTALL_ROOT"
fi

if ! mkdir -p "$INSTALL_ROOT" 2>/dev/null; then
    if [ -n "$SUDO" ]; then
        $SUDO mkdir -p "$INSTALL_ROOT"
    else
        INSTALL_ROOT="$USER_INSTALL_ROOT"
        mkdir -p "$INSTALL_ROOT"
    fi
fi

TMP_DIR=$(mktemp -d)
cleanup() {
    rm -rf "$TMP_DIR"
}
trap cleanup EXIT

WORK_RUN=""
case "$INSTALLER" in
    *.zip)
        require_cmd unzip
        unzip -q "$INSTALLER" -d "$TMP_DIR"
        WORK_RUN=$(find "$TMP_DIR" -type f -name 'SetupSTM32CubeProgrammer*linux*' | head -n 1 || true)
        ;;
    *.tar.gz|*.tgz)
        require_cmd tar
        tar -xf "$INSTALLER" -C "$TMP_DIR"
        WORK_RUN=$(find "$TMP_DIR" -type f -name 'SetupSTM32CubeProgrammer*linux*' | head -n 1 || true)
        ;;
    *linux*)
        WORK_RUN="$INSTALLER"
        ;;
    *)
        echo "Error: unsupported installer format: $INSTALLER"
        exit 1
        ;;
esac

if [ -z "$WORK_RUN" ] || [ ! -f "$WORK_RUN" ]; then
    echo "Error: cannot find Linux installer executable in package."
    exit 1
fi

chmod +x "$WORK_RUN"

echo "Installing STM32CubeProgrammer (silent mode)..."
"$WORK_RUN" --mode unattended --installdir "$INSTALL_ROOT"

BIN_DIR="${INSTALL_ROOT}/bin"
if [ ! -x "${BIN_DIR}/STM32_Programmer_CLI" ]; then
    echo "Warning: install finished but CLI not found at ${BIN_DIR}/STM32_Programmer_CLI"
    exit 1
fi

BASHRC="${HOME}/.bashrc"
LINE="export PATH=\$PATH:${BIN_DIR}"
if ! grep -F "$LINE" "$BASHRC" >/dev/null 2>&1; then
    echo "$LINE" >> "$BASHRC"
fi

export PATH="$PATH:${BIN_DIR}"

echo "Installed successfully."
echo "CLI path: $(command -v STM32_Programmer_CLI)"
STM32_Programmer_CLI --version || true
