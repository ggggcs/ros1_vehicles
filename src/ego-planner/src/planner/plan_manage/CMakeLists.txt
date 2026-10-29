cmake_minimum_required(VERSION 3.5)
project(ego_planner)

set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++11 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")
add_definitions(-DEIGEN_MAX_STATIC_ALIGN_BYTES=0)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)
# 实车(aarch64/JetPack)移植修复, 见下方 ego_planner_node 的 target_link_libraries 注释
find_package(OpenCV REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  quadrotor_msgs
  plan_env
  path_searching
  bspline_opt
  traj_utils
  message_generation
  cv_bridge
  teb_local_planner
)

# Generate messages in the 'msg' folder
add_message_files(
  FILES
  Bspline.msg
  DataDisp.msg
  )
  
# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

# catkin_package(CATKIN_DEPENDS message_runtime)
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ego_planner
 CATKIN_DEPENDS plan_env path_searching bspline_opt traj_utils message_runtime
#  DEPENDS system_lib
)

include_directories(
  include
  SYSTEM
  ${catkin_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)


add_executable(ego_planner_node
  src/ego_planner_node.cpp
  src/ego_replan_fsm.cpp
  src/planner_manager.cpp
  )
# 实车(aarch64/JetPack)移植修复:
#   plan_env 的 grid_map.h 中 MappingData 含 cv::Mat 成员, 本包可执行文件会实例化它,
#   而 plan_env 的 catkin_package() 未导出 OpenCV, 故需在此显式链接。
#   Jetson 上的坑: JetPack 的 libopencv-dev(4.5.4) 覆盖了 /usr/include/opencv4 头文件,
#   而 ROS 的 cv_bridge 是按 OpenCV 4.2 编译并导出 4.2 的 .so。4.2 里 cv::Mat::Mat()
#   是 header inline, 4.5.4 里改成了导出符号 -> 用 4.5.4 头 + 链 4.2 库就会出现
#   undefined reference to `cv::Mat::Mat()'。系统上已无 4.2 头文件, 只能按 4.5.4 头
#   配 4.5.4 库(即 find_package(OpenCV) 的结果), 这里显式链接 ${OpenCV_LIBS}。
#   注: cv_bridge(4.2) 仍会被载入同一进程, 但其 cv::Mat 接口只在深度相机路径
#   (depthOdomCallback/toCvCopy) 用到; 激光模式下 /grid_map/depth 无人发布,
#   message_filters 同步器不触发, 不存在 cv::Mat 跨 ABI 传递。
target_link_libraries(ego_planner_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  )
add_dependencies(ego_planner_node ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(traj_server src/traj_server.cpp)
target_link_libraries(traj_server ${catkin_LIBRARIES})
add_dependencies(traj_server ${${PROJECT_NAME}_EXPORTED_TARGETS})

# TEB 闭环跟踪节点: 订阅 ego-planner B-spline, 最近点投影 + TEB 优化输出 /cmd_vel
add_executable(teb_tracking src/teb_tracking.cpp)
target_link_libraries(teb_tracking ${catkin_LIBRARIES})
add_dependencies(teb_tracking ${${PROJECT_NAME}_EXPORTED_TARGETS})

# ---------------------------------------------------------------------------
# 实车最小地面分割节点 (可选目标)
# ---------------------------------------------------------------------------
# 复用 obs_fitting 里已调好的 patchwork++ header-only 实现, 只做地面分割,
# 不拉起 obs_fitting 的 CUDA 聚类/拟合/跟踪整条管线。**不修改 obs_fitting**,
# 这里只是把它的 include 目录加进来。找不到就跳过该目标, 不影响其它节点编译。
find_path(PATCHWORKPP_INCLUDE_DIR
  NAMES patchworkpp/patchworkpp.hpp
  HINTS ${CMAKE_SOURCE_DIR}/obs_fitting/include
        ${CMAKE_SOURCE_DIR}/../src/obs_fitting/include
  NO_DEFAULT_PATH
)
find_package(jsk_recognition_msgs QUIET)

if(PATCHWORKPP_INCLUDE_DIR AND jsk_recognition_msgs_FOUND)
  message(STATUS "ego_planner: 找到 patchwork++ 头文件于 ${PATCHWORKPP_INCLUDE_DIR}, 编译 patchworkpp_node")
  add_executable(patchworkpp_node src/patchworkpp_node.cpp)
  target_include_directories(patchworkpp_node SYSTEM PRIVATE
    ${PATCHWORKPP_INCLUDE_DIR}
    ${jsk_recognition_msgs_INCLUDE_DIRS}
  )
  target_link_libraries(patchworkpp_node
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
  )
  add_dependencies(patchworkpp_node ${${PROJECT_NAME}_EXPORTED_TARGETS})

  # -- 可选 GPU ROI 裁剪 --------------------------------------------------
  # 复用 obs_fitting 自带的 NVIDIA cudaFilter 库(与它 gpuCropAndDownsample 用的同一个),
  # 只链接不修改。找不到 CUDA 或该库时自动退化为纯 CPU 裁剪, 保证 x86 仿真机也能编。
  # 注意: obs_fitting/CMakeLists.txt 里 CUDA_FILTER_LIB / CUDA_CLUSTER_LIB 两个变量的
  # 路径是写反的; 实际导出 cudaFilter 符号的是 libcudafilter.so。
  find_package(CUDA QUIET)
  find_library(CUDA_POINT_FILTER_LIB
    NAMES cudafilter
    HINTS ${PATCHWORKPP_INCLUDE_DIR}/lib
    NO_DEFAULT_PATH
  )
  find_path(CUDA_POINT_FILTER_INC
    NAMES cudaFilter.h
    HINTS ${PATCHWORKPP_INCLUDE_DIR}/lib
    NO_DEFAULT_PATH
  )
  if(CUDA_FOUND AND CUDA_POINT_FILTER_LIB AND CUDA_POINT_FILTER_INC)
    message(STATUS "ego_planner: 启用 GPU ROI 裁剪 (${CUDA_POINT_FILTER_LIB})")
    get_filename_component(CUDA_POINT_FILTER_DIR ${CUDA_POINT_FILTER_LIB} DIRECTORY)
    target_compile_definitions(patchworkpp_node PRIVATE USE_CUDA_FILTER)
    target_include_directories(patchworkpp_node SYSTEM PRIVATE
      ${CUDA_INCLUDE_DIRS} ${CUDA_POINT_FILTER_INC})
    target_link_libraries(patchworkpp_node ${CUDA_POINT_FILTER_LIB} ${CUDA_LIBRARIES})
    # libcudafilter.so 不在系统库路径里, 需要 rpath 才能在运行时找到
    target_link_options(patchworkpp_node PRIVATE
      "SHELL:-Wl,-rpath,${CUDA_POINT_FILTER_DIR}")
  else()
    message(STATUS "ego_planner: 未找到 CUDA 或 libcudafilter.so, patchworkpp_node 用 CPU 裁剪")
  endif()
else()
  message(WARNING "ego_planner: 未找到 patchwork++ 头文件或 jsk_recognition_msgs, "
                  "跳过 patchworkpp_node (实车需要它做地面分割)")
endif()


