cmake_minimum_required(VERSION 3.0)
project(conti_radar)

#find_package(roslib QUIET)
#include_directories(${roscpp_INCLUDE_DIRS} ${roslib_INCLUDE_DIRS})
#set(ROS_LIBS ${roscpp_LIBRARIES} ${roslib_LIBRARIES})

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    sensor_msgs
    std_msgs
    message_generation
    rosbag
    conti_radar_msgs
#pcl_ros
  )

# find_package(PCL REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime conti_radar_msgs
)

set(CMAKE_BUILD_TYPE "Release")

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_definitions(-DUSE_ROS1)

add_executable(${PROJECT_NAME}_node
    "")

target_compile_options(${PROJECT_NAME}_node
  PRIVATE $<$<CXX_COMPILER_ID:GNU>:-Wall>)

target_sources(${PROJECT_NAME}_node
  PRIVATE
  src/main.cc
  src/radar_can.cc
  ${CMAKE_SOURCE_DIR}/common/net/can/socket_can.cc
    )

target_include_directories(${PROJECT_NAME}_node
    PUBLIC
    /usr/include/glog
    ${catkin_INCLUDE_DIRS}
#  ${PCL_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )

target_link_libraries(${PROJECT_NAME}_node
  ${ROS_LIBS}
  ${catkin_LIBRARIES}
  glog
  )
