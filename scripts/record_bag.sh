#! /bin/bash

DATE_PATH=$(date +%Y%m%d)
TIME_HEAD=$(date +%Y%m%d%H%M%S)

#source install/setup.bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
#source ~/workspace/radar/Radar_test_ws/install/setup.bash
source /home/nvidia/workspace/radar/4DIR_ROS_SHOW_0.0.1_20230926_ubuntu20.04_arm/install/setup.bash

#export ROS_LOCALHOST_ONLY=1
echo ""

if [ ! -d ~/dev/data/$DATE_PATH ]; then
  mkdir ~/dev/data/$DATE_PATH
fi

#RECORD_TOOL="ros2 bag"
RECORD_TOOL="rosbag"
$RECORD_TOOL record -b 2048 /rslidar_points \
       /hnu/sensor/camera_left \
       /hnu/sensor/camera_right \
       /hnu/sensor/conti_radar \
       /hnu/sensor/best_gnss_pos \
       /hnu/sensor/headinga \
       /hnu/sensor/inspvaxa \
       /hnu/sensor/rawimu \
       /Radar_od_1 \
       /Radar_point_1 \
       /sync/conti_radar \
       /sync/inspvaxa \
       /sync/lidar \
       /sync/camera_left \
       /sync/radar \
       -o ~/dev/data/$DATE_PATH/$TIME_HEAD
