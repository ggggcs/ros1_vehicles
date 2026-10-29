cmake_minimum_required(VERSION 3.0)
project(remote_control)

# 设置C++标准
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_definitions(-DUSE_ROS1)

# 查找依赖的ROS 1软件包
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
)

# 声明catkin包
catkin_package(
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs
)

# 包含头文件目录
include_directories(
  ${catkin_INCLUDE_DIRS}
)

# 添加可执行文件
add_executable(${PROJECT_NAME}_node src/main.cc)

# 设置可执行文件链接的依赖库
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)

# 设置依赖关系
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
