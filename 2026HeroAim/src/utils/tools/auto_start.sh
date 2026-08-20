#!/bin/bash

sec="${AUTOAIM_RESTART_SEC:-5}"
path="${AUTOAIM_PATH:-/home/nvidia/2026HeroAim/build}"
name="${AUTOAIM_NAME:-AutoAim}"

cd "$path" || {
    echo "Failed to cd into $path"
    exit 1
}

set_serial_permissions() {
    chmod 666 /dev/ttyACM0 2>/dev/null || true
    chmod 666 /dev/ttyACM1 2>/dev/null || true
}

trap 'echo "auto_start stopped"; exit 0' SIGINT SIGTERM

while true; do
    echo "Starting $name..."
    set_serial_permissions

    # Keep AutoAim in the foreground so Bash waits for and reaps this exact
    # child.  Do not use pgrep: it also matches a terminated zombie process.
    ./"$name"
    exit_code=$?

    echo "$name exited with code $exit_code; restarting in ${sec}s..."
    sleep "$sec"
done
