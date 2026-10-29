#!/usr/bin/env bash

set -u

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PTP_CFG="$ROOT_DIR/gPTP.cfg"
PTP_IFACE="eth1"
PTP_DEVICE="/dev/ptp1"
PTP4L_LOG="/tmp/ptp4l-eth1.log"
PHC2SYS_LOG="/tmp/phc2sys-eth1.log"

run_privileged() {
    if [ "${EUID}" -eq 0 ]; then
        "$@"
    else
        sudo "$@"
    fi
}

if [ ! -f "$PTP_CFG" ]; then
    echo "PTP config not found: $PTP_CFG" >&2
    exit 1
fi

for _ in $(seq 1 20); do
    if [ -e "/sys/class/net/$PTP_IFACE" ] && [ -e "$PTP_DEVICE" ]; then
        break
    fi
    sleep 1
done

if [ ! -e "/sys/class/net/$PTP_IFACE" ] || [ ! -e "$PTP_DEVICE" ]; then
    echo "PTP interface/device not ready: $PTP_IFACE $PTP_DEVICE" >&2
    exit 1
fi

if command -v systemctl >/dev/null && [ -f /etc/systemd/system/ptp4l-eth1.service ]; then
    run_privileged systemctl start ptp4l-eth1.service phc2sys-eth1.service
else
    if ! pgrep -f '(^|/)ptp4l .*eth1' >/dev/null; then
        run_privileged nohup /usr/sbin/ptp4l -i "$PTP_IFACE" -m -f "$PTP_CFG" \
            >"$PTP4L_LOG" 2>&1 </dev/null &
    fi

    sleep 1

    if ! pgrep -f '[/]phc2sys .*[/]dev/ptp1' >/dev/null; then
        run_privileged nohup /usr/sbin/phc2sys -s CLOCK_REALTIME -c "$PTP_DEVICE" -O 0 -m \
            >"$PHC2SYS_LOG" 2>&1 </dev/null &
    fi
fi

echo "PTP startup requested: $PTP_IFACE -> $PTP_DEVICE"
