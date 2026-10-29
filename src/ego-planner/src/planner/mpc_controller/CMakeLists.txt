cmake_minimum_required(VERSION 3.5)
project(mpc_controller)

set(CMAKE_BUILD_TYPE "Release")
add_compile_options(-std=c++14)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")
add_definitions(-DEIGEN_MAX_STATIC_ALIGN_BYTES=0)

find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES mpc_controller
  CATKIN_DEPENDS roscpp
  DEPENDS Eigen3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIR}
  ${Eigen3_INCLUDE_DIRS}
)

# MPC library (currently no-op without qpOASES, headers only)
# To enable full MPC: build qpOASES manually, then add:
#   include_directories(qpOASES/include)
#   find_library(QPOASES_LIB qpOASES)
#   target_link_libraries(mpc_controller ${QPOASES_LIB})
add_library(mpc_controller
  src/uniform_bspline.cpp
)

target_link_libraries(mpc_controller
  ${catkin_LIBRARIES}
)
