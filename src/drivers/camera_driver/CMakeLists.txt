cmake_minimum_required(VERSION 3.0)
project(camera_driver)

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
#pcl_ros
  )

# find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
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

#add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME}_generate_messages_cpp)

#set(sub_src src/sub.cc)
#set(sub_src src/sub_all.cc)

target_sources(${PROJECT_NAME}_node
  PRIVATE
  src/main.cc
  ${sub_src}
  src/save_image.cc
  common/src/cfg.cpp
  )

target_include_directories(${PROJECT_NAME}_node
    PUBLIC
    ${catkin_INCLUDE_DIRS}
#  ${PCL_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/common/include
  )

find_library(CAM_LIBS libsyncv4l2.so ${CMAKE_CURRENT_SOURCE_DIR}/lib)

target_link_libraries(${PROJECT_NAME}_node
  ${CAM_LIBS}
  ${ROS_LIBS}
  ${catkin_LIBRARIES}
	syncv4l2 pthread ${OpenCV_LIBS}
  )
