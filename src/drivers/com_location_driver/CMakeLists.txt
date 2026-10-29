cmake_minimum_required(VERSION 3.0)
project(com_location_driver)

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
    com_location_msgs
#pcl_ros
  )

# find_package(PCL REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime com_location_msgs
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
  src/tcp_client.cc
  src/data_parser.cc
  src/gnss_data.cc
    )

target_include_directories(${PROJECT_NAME}_node
    PUBLIC
    ${catkin_INCLUDE_DIRS}
#  ${PCL_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )

find_library(COM_LIBS libhv.a ${CMAKE_CURRENT_SOURCE_DIR}/lib)

target_link_libraries(${PROJECT_NAME}_node
  ${COM_LIBS}
  ${ROS_LIBS}
  ${catkin_LIBRARIES}
  )
