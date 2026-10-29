#! /bin/bash

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/.."

# 相机初始化
sudo chmod 777 /dev/ttyTHS1
sudo systemctl restart jetson-cam-cfg.service
#sudo tztek-jetson-tool-internal-trigger-camera /dev/ttyTHS1 10 1000
#sudo mount /dev/nvme0n1p1 /media/disk

bash $ROOT_DIR/scripts/start_can.sh

bash $ROOT_DIR/scripts/start_ptp.sh

ifconfig eth5 down

# 室内
#sudo tztek-jetson-tool-cpld-test -d /dev/ttyTHS1 -t 2 -l 232 -b 9600
# 室外
#sudo tztek-jetson-tool-cpld-test -d /dev/ttyTHS1 -t 3 -l 232 -b 9600
echo "" > /tmp/start_sys
# LOG COM1 GPRMC ONTIME 1
# SAVECONFIG

# NTRIPCONFIG NCOM1 CLIENT V1 120.253.226.97:8002 RTCM33_GRCEJ cedr20449 6xuf2hjr ALL
# INTERFACEMODE NCOM1 RTCM BYNAV
# LOG NTRIPCONFIG ONCE
# UNLOG INSPVAXA
