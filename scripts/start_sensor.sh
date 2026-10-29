#! /bin/bash
  
cam_res=$(pidof camera_driver_node)
lidar_res=$(pidof rslidar_sdk_node)
stereo_radar_res=$(ps aux |grep "launch radar_show" | head -n 1 | awk '{print $2}')
radar_res=$(pidof conti_radar_node)
imu_res=$(pidof com_location_driver_node)
canbus_res=$(pidof canbus_node)
joy_res=$(pidof joy_node)

run_node()
{
  LOG_PAH="./log"/$(date +%Y%m%d)
  if [ ! -d $LOG_PAH ]; then
    mkdir -p $LOG_PAH
  fi

  log_file=$LOG_PAH/$2"_"$(date +%Y%m%d_%H%M%S)".log"
  $1 > $log_file
}

start_node_with_log()
{
  #source install/setup.bash
  /opt/ros/noetic/setup.bash
  source devel/setup.bash

  # camera
  if [ "$cam_res" == "" ]; then
    run_node ./build/camera_driver/camera_driver camera
  fi

  # lidar
  if [ "$lidar_res" == "" ]; then
    run_node "ros2 launch rslidar_sdk start.py" lidar
  fi

  if [ "$stereo_radar_res" == "" ]; then
    # stereo radar(厂家因为我们的量小不提供sdk，所以单独启动)
    #run_node "cd ~/workspace/radar/Radar_test_ws && bash ./radar_single.sh" 4d_radar
    echo ""
  fi

  # radar
  if [ "$radar_res" == "" ]; then
    run_node ./build/conti_radar_node/conti_radar_node conti_radar
  fi

  # rtk+imu
  if [ "$imu_res" == "" ]; then
    run_node ./build/com_location_driver/com_location_driver com_location
  fi

  ros2 run joy joy_node > /dev/null &
  # canbbus
  if [ "$canbus_res" == "" ]; then
    run_node ./build/canbus/canbus canbus > /dev/null
  fi
}

start_node_without_log()
{
#source install/setup.bash
  source /opt/ros/noetic/setup.bash
source devel/setup.bash

  # camera
  #./build/camera_driver/camera_driver &
  ./devel/lib/camera_driver/camera_driver_node &

  # lidar
  #ros2 launch rslidar_sdk start.py &
  roslaunch rslidar_sdk start.launch
  #./devel/lib/rslidar_sdk/rslidar_sdk_node

  # stereo radar(厂家因为我们的量小不提供sdk，所以单独启动)
  #  cd ~/workspace/radar/Radar_test_ws && bash ./radar_single.sh &
     cd ~/workspace/radar/4DIR_ROS_SHOW_0.0.1_20230926_ubuntu20.04_arm  &&  bash ./radar_show_single.sh

  # radar
  #./build/conti_radar_node/conti_radar_node &
  ./devel/lib/conti_radar/conti_radar_node

  # rtk+imu
  #./build/com_location_driver/com_location_driver &
  ./devel/lib/com_location_driver/com_location_driver_node &

  #ros2 run joy joy_node > /dev/null &
  #rosrun joy joy_node > /dev/null &
  /opt/ros/noetic/lib/joy/joy_node > /dev/null &

  # remote control
  ./devel/lib/remote_control/remote_control_node > /dev/null &
  
  # pano camera
  ./devel/lib/rocon_rtsp_camera_relay/rocon_rtsp_camera_relay_node

  #sync_data
  ./devel/lib/sync_data/sync_data_node
}

stop_node()
{
  kill -9 $cam_res
  kill -9 $lidar_res
  kill -9 $(ps aux |grep "launch radar_show" | head -n 1 | awk '{print $2}')
  radar_res=$(ps aux |grep Radar_test_ws | awk '{print $2}')
  for i in $radar_res; do kill -9 $i; done
  kill -9 $imu_res
  for i in $joy_res; do kill -9 $i; done
  kill -9 $canbus_res
}

if [ "$1" == "start" ]; then
  start_node_without_log
elif [ "$1" == "stop" ]; then
  stop_node
else
  echo "Usage: $0 start|stop"
fi

exit 0
