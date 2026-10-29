cmake_minimum_required(VERSION 3.0)
project(sync_data)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_BUILD_TYPE "Release")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    sensor_msgs
    std_msgs
    geometry_msgs
    radar_msgs
    message_filters
    rosbag
  )

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs radar_msgs)

add_executable(${PROJECT_NAME}_node "")

target_compile_options(${PROJECT_NAME}_node
  PRIVATE $<$<CXX_COMPILER_ID:GNU>:-Wall>)

target_sources(${PROJECT_NAME}_node
  PRIVATE
  src/ros1_all.cc)

target_include_directories(${PROJECT_NAME}_node
    PUBLIC
    /usr/include/glog
    ${catkin_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}/../devel/include
    ${CMAKE_CURRENT_SOURCE_DIR}/
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )

target_link_libraries(${PROJECT_NAME}_node
  ${ROS_LIBS}
  ${catkin_LIBRARIES}
  glog
  )
