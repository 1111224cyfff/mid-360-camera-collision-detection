cmake_minimum_required(VERSION 3.1)
project(yolo11_ros)

# ========== 关键修改：OpenCV 4.5 配置 START ==========
# 1. 优先使用本地 OpenCV 4.5，自定义路径不存在时回退到系统安装
set(_opencv45_local "/usr/local/opencv45/lib/cmake/opencv4")
set(_opencv45_system "/usr/lib/aarch64-linux-gnu/cmake/opencv4")

if(EXISTS "${_opencv45_local}/OpenCVConfig.cmake")
  set(OpenCV_DIR "${_opencv45_local}")
elseif(EXISTS "${_opencv45_system}/OpenCVConfig.cmake")
  set(OpenCV_DIR "${_opencv45_system}")
endif()

# 2. 查找 OpenCV 4.5
find_package(OpenCV 4.5 REQUIRED)

# 3. 包含 OpenCV 头文件
include_directories(${OpenCV_INCLUDE_DIRS})

# 4. 打印调试信息，确保找到的是正确的版本和路径
message(STATUS "========== OpenCV Configuration ==========")
message(STATUS "  Version: ${OpenCV_VERSION}")
message(STATUS "  Include: ${OpenCV_INCLUDE_DIRS}")
message(STATUS "  Libs:    ${OpenCV_LIBS}")
message(STATUS "==========================================")

# 5. 保留基础编译选项
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -O3")
# ========== 关键修改：OpenCV 4.5 配置 END ============

# 配置 TensorRT 路径
set(TENSORRT_ROOT /opt/TensorRT-8.5.1.7)
include_directories(${TENSORRT_ROOT}/include)
link_directories(${TENSORRT_ROOT}/lib)

# 配置 CUDA（适配低版本 CMake + Jetson aarch64架构）
set(CMAKE_CUDA_ARCHITECTURES 53 62 72 87)  # 适配Jetson Nano/Xavier/NX
set(CMAKE_CUDA_COMPILER /usr/local/cuda-11.4/bin/nvcc)
find_package(CUDA 11.4 REQUIRED)  # 使用旧版 CUDA 查找方式

# 编译选项
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE Release)
option(CUDA_USE_STATIC_CUDA_RUNTIME OFF)

# ========== 关键修改2：修正CUDA库路径（Jetson是aarch64，不是x86_64） ==========
# 路径配置
set(CUDA_INCLUDE_DIRS ${CUDA_INCLUDE_DIRS} /usr/local/cuda-11.4/include)
set(CUDA_LIBRARIES_DIR /usr/local/cuda-11.4/targets/aarch64-linux/lib)  # 修正为aarch64
set(TensorRT_INCLUDE_DIRS ${TENSORRT_ROOT}/include)
set(TensorRT_LIBRARIES ${TENSORRT_ROOT}/lib)

# 打印路径信息（用于调试）
message(STATUS "CUDA Headers: \n${CUDA_INCLUDE_DIRS}\n")
message(STATUS "CUDA Libs Dir: \n${CUDA_LIBRARIES_DIR}\n")
message(STATUS "TensorRT Headers: \n${TensorRT_INCLUDE_DIRS}\n")
message(STATUS "TensorRT Libs: \n${TensorRT_LIBRARIES}\n")

# 寻找其他依赖包（保留）
find_package(PCL REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  rospy
  cv_bridge
  message_filters
  tf2
  tf2_ros
  sensor_msgs
)

# Catkin 配置
catkin_package()

# 包含头文件路径
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${CUDA_INCLUDE_DIRS}
  ${TensorRT_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

# 链接库路径
link_directories(
  ${PCL_LIBRARY_DIRS}
  ${CUDA_LIBRARIES_DIR}
  ${TensorRT_LIBRARIES}
)

add_compile_options(-Wno-deprecated-declarations)
add_definitions(${PCL_DEFINITIONS})

set(_yolo11_root_candidates
  "${CMAKE_CURRENT_SOURCE_DIR}/TensorRT-YOLO11/C++"
)

set(YOLO11_ROOT "")
foreach(_yolo11_root_candidate IN LISTS _yolo11_root_candidates)
  get_filename_component(_yolo11_root_candidate_abs "${_yolo11_root_candidate}" ABSOLUTE)
  if(EXISTS "${_yolo11_root_candidate_abs}")
    set(YOLO11_ROOT "${_yolo11_root_candidate_abs}")
    break()
  endif()
endforeach()

if(YOLO11_ROOT STREQUAL "")
  message(FATAL_ERROR "TensorRT-YOLO11 source directory not found. Tried: ${_yolo11_root_candidates}")
endif()

set(YOLO11_DETECT_SRCS
  ${YOLO11_ROOT}/detect/src/calibrator.cpp
  ${YOLO11_ROOT}/detect/src/infer.cpp
  ${YOLO11_ROOT}/detect/src/postprocess.cu
  ${YOLO11_ROOT}/detect/src/preprocess.cu
)

set(YOLO11_POSE_SRCS
  ${YOLO11_ROOT}/pose/src/calibrator.cpp
  ${YOLO11_ROOT}/pose/src/infer.cpp
  ${YOLO11_ROOT}/pose/src/postprocess.cu
  ${YOLO11_ROOT}/pose/src/preprocess.cu
)

set(YOLO11_SEGMENT_SRCS
  ${YOLO11_ROOT}/segment/src/calibrator.cpp
  ${YOLO11_ROOT}/segment/src/draw.cu
  ${YOLO11_ROOT}/segment/src/infer.cpp
  ${YOLO11_ROOT}/segment/src/postprocess.cu
  ${YOLO11_ROOT}/segment/src/preprocess.cu
)

# ------------------------------
# 定义目标并链接库
# ------------------------------

# 1. detect 节点
cuda_add_executable(detect src_ros/detect.cpp ${YOLO11_DETECT_SRCS})
target_include_directories(detect PRIVATE
  ${YOLO11_ROOT}/detect/include
)
target_link_libraries(detect 
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
  ${CUDA_LIBRARIES}  # 链接CUDA库
  nvinfer
  nvinfer_plugin
  nvonnxparser
)

# 2. pose 节点
cuda_add_executable(pose src_ros/pose.cpp ${YOLO11_POSE_SRCS})
target_include_directories(pose PRIVATE
  ${YOLO11_ROOT}/pose/include
)
target_link_libraries(pose 
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
  ${CUDA_LIBRARIES}  # 链接CUDA库
  nvinfer
  nvinfer_plugin
  nvonnxparser
)

# 3. segment 节点
cuda_add_executable(segment src_ros/segment.cpp ${YOLO11_SEGMENT_SRCS})
target_include_directories(segment PRIVATE
  ${YOLO11_ROOT}/segment/include
)
target_link_libraries(segment 
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
  ${CUDA_LIBRARIES}  # 链接CUDA库
  nvinfer
  nvinfer_plugin
  nvonnxparser
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"  # 匹配所有 .launch 文件
)

