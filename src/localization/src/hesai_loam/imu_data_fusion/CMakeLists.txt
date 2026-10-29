cmake_minimum_required(VERSION 3.0.2)
project(imu_data_fusion)

## 找到依赖
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
  tf2_geometry_msgs
  message_filters
  novatel_oem7_msgs
)

## 声明包
catkin_package(
  CATKIN_DEPENDS 
    roscpp 
    sensor_msgs 
    geometry_msgs 
    tf2_geometry_msgs 
    message_filters
    novatel_oem7_msgs
)

## Include 目录
include_directories(${catkin_INCLUDE_DIRS})

## 可执行文件
add_executable(imu_data_fusion_node src/imu_data_fusion.cpp)

## 依赖
add_dependencies(imu_data_fusion_node 
  ${catkin_EXPORTED_TARGETS}
)

## 链接
target_link_libraries(imu_data_fusion_node ${catkin_LIBRARIES})
