cmake_minimum_required(VERSION 3.0)
project(obs_fitting)

## Compile as C++14
add_compile_options(-std=c++14)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  pcl_conversions
  pcl_ros
  sensor_msgs
  visualization_msgs
  cv_bridge
  tf2_ros
  geometry_msgs
  message_generation
)

# 自动检测CUDA
execute_process(
    COMMAND bash -c "dpkg -l | grep cuda-core | sed -e 's/ \\{1,\\}/ /g' | cut -d ' ' -f 3 | cut -d '.' -f 1,2 | sed -e 's/-.*//g' | sort -n | tail -n 1"
    OUTPUT_VARIABLE DETECTED_CUDA_VERSION
    OUTPUT_STRIP_TRAILING_WHITESPACE
)

if(DETECTED_CUDA_VERSION)
    set(CUDAVERSION "cuda-${DETECTED_CUDA_VERSION}")
    set(CUDA_DIR "/usr/local/${CUDAVERSION}")
else()
    set(CUDAVERSION "cuda")
    set(CUDA_DIR "/usr/local/cuda")
endif()

if(NOT EXISTS "${CUDA_DIR}")
    message(FATAL_ERROR "CUDA not found at ${CUDA_DIR}")
endif()

message(STATUS "Using CUDA directory: ${CUDA_DIR}")

find_package(CUDA REQUIRED)

# +++ 直接指定 cudaFilter 和 cudaCluster 库路径 +++
set(CUDA_FILTER_LIB "/home/nvidia/workspace/ros1_vehicles/src/obs_fitting/include/lib/libcudacluster.so")
set(CUDA_CLUSTER_LIB "/home/nvidia/workspace/ros1_vehicles/src/obs_fitting/include/lib/libcudafilter.so")

# 检查cudaFilter库
if(NOT EXISTS ${CUDA_FILTER_LIB})
    message(FATAL_ERROR "cudaFilter library not found at: ${CUDA_FILTER_LIB}")
else()
    message(STATUS "Using cudaFilter library: ${CUDA_FILTER_LIB}")
    get_filename_component(CUDA_FILTER_LIB_DIR ${CUDA_FILTER_LIB} DIRECTORY)
    message(STATUS "cudaFilter library directory: ${CUDA_FILTER_LIB_DIR}")
endif()

# 检查cudaCluster库
if(NOT EXISTS ${CUDA_CLUSTER_LIB})
    message(WARNING "cudaCluster library not found at: ${CUDA_CLUSTER_LIB}")
    message(WARNING "Will try to find it in standard library paths")
    
    find_library(CUDA_CLUSTER_LIB 
        NAMES cudacluster cudaCluster
        PATHS 
            /usr/local/lib
            /usr/lib
            ${CUDA_DIR}/lib64
            /home/nvidia/workspace/ros1_vehicles/src/obs_fitting/include/lib
    )
    
    if(CUDA_CLUSTER_LIB)
        message(STATUS "Found cudaCluster library: ${CUDA_CLUSTER_LIB}")
    else()
        message(FATAL_ERROR "cudaCluster library not found. Please build it first.")
    endif()
else()
    message(STATUS "Using cudaCluster library: ${CUDA_CLUSTER_LIB}")
endif()

get_filename_component(CUDA_CLUSTER_LIB_DIR ${CUDA_CLUSTER_LIB} DIRECTORY)

# 设置链接器标志
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--allow-shlib-undefined -pthread")

find_package(OpenCV REQUIRED)

find_package(OpenMP REQUIRED)
if (OPENMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

find_package(Eigen3 REQUIRED)

# 设置PCL相关库
set(PCL_LIBRARIES_EXTRA
    pcl_common pcl_io pcl_recognition pcl_features 
    pcl_sample_consensus pcl_octree pcl_search 
    pcl_filters pcl_kdtree pcl_segmentation 
    pcl_visualization
)

find_package(Boost REQUIRED COMPONENTS system)

## ===== 生成自定义消息 =====
add_message_files(
  FILES
  CarEllipse.msg
  CarEllipseArray.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

###################################
## catkin_package 配置
###################################
catkin_package(
  CATKIN_DEPENDS 
    roscpp 
    sensor_msgs 
    pcl_conversions 
    pcl_ros 
    visualization_msgs 
    cv_bridge
    tf2_ros
    geometry_msgs
    message_runtime
)

###########
## Build ##
###########

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${CUDA_INCLUDE_DIRS}
    ${CUDA_DIR}/include
    /usr/local/include
    /usr/include/eigen3
    ${Eigen3_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}/include
)

link_directories(
    ${CUDA_LIBRARY_DIRS}
    ${CUDA_DIR}/lib64
    ${PCL_LIBRARY_DIRS}
    ${CUDA_FILTER_LIB_DIR}
    ${CUDA_CLUSTER_LIB_DIR}
)

# ARM架构优化标志
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
    message(STATUS "✅ Detected ARM architecture, enabling optimizations")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native -O3 -ftree-vectorize")
endif()

## ===== 主处理节点 =====
add_executable(main src/main.cpp)

target_link_libraries(main
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    ${PCL_LIBRARIES_EXTRA}
    ${OpenCV_LIBS}
    ${OpenMP_CXX_LIBRARIES}
    ${CUDA_LIBRARIES}
    ${CUDA_CUDART_LIBRARY}
    ${Boost_LIBRARIES}
    ${CUDA_FILTER_LIB}
    ${CUDA_CLUSTER_LIB}
    cudart
    rt
    dl
    pthread
    stdc++fs
)

target_link_options(main PRIVATE 
    "SHELL:-Wl,-rpath,${CUDA_FILTER_LIB_DIR}"
    "SHELL:-Wl,-rpath,${CUDA_CLUSTER_LIB_DIR}"
    "SHELL:-Wl,-rpath,${CUDA_DIR}/lib64"
)

set_target_properties(main PROPERTIES
    CXX_STANDARD 14
    POSITION_INDEPENDENT_CODE ON
)

# 添加消息依赖
add_dependencies(main ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## ===== 椭圆可视化节点 =====
add_executable(ellipse_visualizer src/ellipse_visualizer.cpp)

target_link_libraries(ellipse_visualizer
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

# 添加消息依赖
add_dependencies(ellipse_visualizer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#############
## Install ##
#############
install(TARGETS main ellipse_visualizer
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)