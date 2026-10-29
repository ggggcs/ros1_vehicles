cmake_minimum_required(VERSION 3.0.2)
project(lidar_imu_sync)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  message_filters
  novatel_oem7_msgs
)

catkin_package(
  CATKIN_DEPENDS 
    roscpp 
    sensor_msgs
    message_filters
    novatel_oem7_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# 激光雷达-IMU同步节点
add_executable(lidar_imu_sync_node src/lidar_imu_sync.cpp)
target_link_libraries(lidar_imu_sync_node ${catkin_LIBRARIES})

install(TARGETS 
  lidar_imu_sync_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
