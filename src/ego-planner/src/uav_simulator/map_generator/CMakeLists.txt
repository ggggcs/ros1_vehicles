cmake_minimum_required(VERSION 3.5)
project(map_generator)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  pcl_conversions
)

find_package(PCL 1.10 REQUIRED COMPONENTS common io kdtree search filters)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem thread)

# 添加必要的定义
add_definitions(${PCL_DEFINITIONS})
add_definitions(-DQT_NO_KEYWORDS)

catkin_package(
  DEPENDS 
    PCL
    Eigen3
    Boost
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

# 链接库变量
set(LIBS 
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${Boost_LIBRARIES}
  pcl_kdtree
  pcl_common
  pcl_search
  flann_cpp
)

add_executable(random_forest src/random_forest_sensing.cpp)

target_link_libraries(random_forest ${LIBS})