cmake_minimum_required(VERSION 2.8.3)

project(hikrobot_camera)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
  sensor_msgs
  tf2
  tf2_ros
  cv_bridge
  image_transport
  camera_info_manager
)

catkin_package(
 
)

find_package(OpenCV 4.5.0 REQUIRED)

set(MVS_ROOT "/opt/MVS" CACHE PATH "Hikrobot MVS SDK root directory")

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  include
  ${MVS_ROOT}/include/
)

if (EXISTS "${MVS_ROOT}/lib/64")
  set(MVS_LIB_DIR "${MVS_ROOT}/lib/64")
elseif (EXISTS "${MVS_ROOT}/lib/aarch64")
  set(MVS_LIB_DIR "${MVS_ROOT}/lib/aarch64")
elseif (EXISTS "${MVS_ROOT}/lib/arm64")
  set(MVS_LIB_DIR "${MVS_ROOT}/lib/arm64")
else()
  message(FATAL_ERROR "Hikrobot MVS SDK libraries not found under ${MVS_ROOT}/lib. Set -DMVS_ROOT=/path/to/MVS")
endif()

set(HIK_MVS_LIBS
  GCBase_gcc494_v3_0
  GenCP_gcc494_v3_0
  log4cpp_gcc494_v3_0
  Log_gcc494_v3_0
  MvCameraControl
  MVGigEVisionSDK
  MVRender
  MvUsb3vTL
  MediaProcess
  FormatConversion
)

foreach(lib ${HIK_MVS_LIBS})
  find_library(HIK_MVS_${lib} NAMES ${lib} PATHS ${MVS_LIB_DIR} NO_DEFAULT_PATH)
  if (NOT HIK_MVS_${lib})
    message(FATAL_ERROR "Missing Hikrobot MVS library ${lib} in ${MVS_LIB_DIR}. Set -DMVS_ROOT=/path/to/MVS")
  endif()
endforeach()

add_executable(${PROJECT_NAME}
src/hikrobot_camera.cpp)

add_executable(camera_frame_transform_node
src/camera_frame_transform_node.cpp)

catkin_install_python(PROGRAMS
  scripts/check_camera_lidar_alignment.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
  launch/
  config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

 target_link_libraries(${PROJECT_NAME}
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
   ${HIK_MVS_GCBase_gcc494_v3_0}
   ${HIK_MVS_GenCP_gcc494_v3_0}
   ${HIK_MVS_log4cpp_gcc494_v3_0}
   ${HIK_MVS_Log_gcc494_v3_0}
   ${HIK_MVS_MvCameraControl}
   ${HIK_MVS_MVGigEVisionSDK}
   ${HIK_MVS_MVRender}
   ${HIK_MVS_MvUsb3vTL}
   ${HIK_MVS_MediaProcess}
   ${HIK_MVS_FormatConversion}
   X11
 )

target_link_libraries(camera_frame_transform_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)


