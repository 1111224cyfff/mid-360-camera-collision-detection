cmake_minimum_required(VERSION 3.1)
project(yolo26_ros)

set(_opencv45_local "/usr/local/opencv45/lib/cmake/opencv4")
set(_opencv45_system "/usr/lib/aarch64-linux-gnu/cmake/opencv4")

if(EXISTS "${_opencv45_local}/OpenCVConfig.cmake")
  set(OpenCV_DIR "${_opencv45_local}")
elseif(EXISTS "${_opencv45_system}/OpenCVConfig.cmake")
  set(OpenCV_DIR "${_opencv45_system}")
endif()

find_package(OpenCV 4.5 REQUIRED)
find_package(PCL REQUIRED)
find_package(CUDA 11.4 REQUIRED)
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

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
list(APPEND CUDA_NVCC_FLAGS "-std=c++17")
option(CUDA_USE_STATIC_CUDA_RUNTIME OFF)

set(TENSORRT_ROOT /opt/TensorRT-8.5.1.7)
set(CMAKE_CUDA_COMPILER /usr/local/cuda-11.4/bin/nvcc)
set(CMAKE_CUDA_ARCHITECTURES 53 62 72 87)

set(CUDA_LIBRARIES_DIR /usr/local/cuda-11.4/targets/aarch64-linux/lib)
set(TensorRT_INCLUDE_DIRS ${TENSORRT_ROOT}/include)
set(TensorRT_LIBRARY_DIRS ${TENSORRT_ROOT}/lib)

set(_yolo26_vendor_root_candidates
  "${CMAKE_CURRENT_SOURCE_DIR}/../yolo-inference/cpp"
)

set(YOLO26_VENDOR_ROOT "")
foreach(_yolo26_vendor_root_candidate IN LISTS _yolo26_vendor_root_candidates)
  get_filename_component(_yolo26_vendor_root_candidate_abs "${_yolo26_vendor_root_candidate}" ABSOLUTE)
  if(EXISTS "${_yolo26_vendor_root_candidate_abs}")
    set(YOLO26_VENDOR_ROOT "${_yolo26_vendor_root_candidate_abs}")
    break()
  endif()
endforeach()

if(YOLO26_VENDOR_ROOT STREQUAL "")
  message(FATAL_ERROR "yolo-inference cpp source directory not found. Tried: ${_yolo26_vendor_root_candidates}")
endif()

catkin_package()

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${CUDA_INCLUDE_DIRS}
  ${TensorRT_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${YOLO26_VENDOR_ROOT}
  ${YOLO26_VENDOR_ROOT}/tensorrt
  ${YOLO26_VENDOR_ROOT}/tensorrt/cuda
)

link_directories(
  ${PCL_LIBRARY_DIRS}
  ${CUDA_LIBRARIES_DIR}
  ${TensorRT_LIBRARY_DIRS}
)

add_compile_options(-Wno-deprecated-declarations)
add_definitions(${PCL_DEFINITIONS})

set(YOLO26_BACKEND_SRCS
  ${YOLO26_VENDOR_ROOT}/utils.cpp
  ${YOLO26_VENDOR_ROOT}/yolo.cpp
  ${YOLO26_VENDOR_ROOT}/tensorrt/yolo_tensorrt.cpp
  ${YOLO26_VENDOR_ROOT}/tensorrt/yolo_tensorrt_detect.cpp
  ${YOLO26_VENDOR_ROOT}/tensorrt/yolo_tensorrt_segment.cpp
  ${YOLO26_VENDOR_ROOT}/tensorrt/yolo_tensorrt_pose.cpp
  ${YOLO26_VENDOR_ROOT}/tensorrt/cuda/preprocess.cu
  ${YOLO26_VENDOR_ROOT}/tensorrt/cuda/decode.cu
)

cuda_add_library(yolo26_tensorrt_backend STATIC ${YOLO26_BACKEND_SRCS})
target_compile_definitions(yolo26_tensorrt_backend PRIVATE _CUDA_PREPROCESS _CUDA_POSTPROCESS)
target_link_libraries(yolo26_tensorrt_backend
  ${OpenCV_LIBS}
  ${CUDA_LIBRARIES}
  nvinfer
  nvinfer_plugin
)

cuda_add_executable(yolo26_detect_node src_ros/detect.cpp)
set_target_properties(yolo26_detect_node PROPERTIES OUTPUT_NAME detect)
target_compile_definitions(yolo26_detect_node PRIVATE YOLO26_TASK_DETECT _CUDA_PREPROCESS _CUDA_POSTPROCESS)
target_link_libraries(yolo26_detect_node
  yolo26_tensorrt_backend
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
  ${CUDA_LIBRARIES}
  nvinfer
  nvinfer_plugin
)

cuda_add_executable(yolo26_pose_node src_ros/pose.cpp)
set_target_properties(yolo26_pose_node PROPERTIES OUTPUT_NAME pose)
target_compile_definitions(yolo26_pose_node PRIVATE YOLO26_TASK_POSE _CUDA_PREPROCESS _CUDA_POSTPROCESS)
target_link_libraries(yolo26_pose_node
  yolo26_tensorrt_backend
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
  ${CUDA_LIBRARIES}
  nvinfer
  nvinfer_plugin
)

cuda_add_executable(yolo26_segment_node src_ros/segment.cpp)
set_target_properties(yolo26_segment_node PROPERTIES OUTPUT_NAME segment)
target_compile_definitions(yolo26_segment_node PRIVATE YOLO26_TASK_SEGMENT _CUDA_PREPROCESS _CUDA_POSTPROCESS)
target_link_libraries(yolo26_segment_node
  yolo26_tensorrt_backend
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
  ${CUDA_LIBRARIES}
  nvinfer
  nvinfer_plugin
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)