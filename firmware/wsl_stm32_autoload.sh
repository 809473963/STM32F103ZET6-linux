#!/usr/bin/env bash
set -euo pipefail

# Auto-load CH34x serial driver and keep ttyUSB/ttyACM writable in WSL.
modprobe usbserial || true
modprobe ch341 || true

apply_permissions() {
    local dev
    for dev in /dev/ttyUSB* /dev/ttyACM*; do
        [[ -e "$dev" ]] || continue
        chmod 666 "$dev" || true
    done
}

apply_permissions

pid_file="/var/run/wsl-stm32-autoload-watch.pid"
if [[ -f "$pid_file" ]]; then
    old_pid="$(cat "$pid_file" 2>/dev/null || true)"
    if [[ -n "$old_pid" ]] && kill -0 "$old_pid" 2>/dev/null; then
        exit 0
    fi
fi

(
    while true; do
        apply_permissions
        sleep 1
    done
) >/dev/null 2>&1 &

echo "$!" > "$pid_file"
