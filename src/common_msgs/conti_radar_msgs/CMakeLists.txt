cmake_minimum_required(VERSION 3.0)
project(conti_radar_msgs)

#find_package(roslib QUIET)
#include_directories(${roscpp_INCLUDE_DIRS} ${roslib_INCLUDE_DIRS})
#set(ROS_LIBS ${roscpp_LIBRARIES} ${roslib_LIBRARIES})

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    sensor_msgs
    std_msgs
    message_generation
    geometry_msgs
    rosbag
  )

# find_package(PCL REQUIRED)

add_message_files(FILES
  ContiTarget.msg
  ContiTargets.msg
  Obstacle.msg
)

generate_messages(DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs message_runtime
)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
