cmake_minimum_required(VERSION 2.8.3)
project(lio_sam)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -pthread")
# set(PCL_DIR "/home/deploy/livox_lidar/pcl-pcl-1.8.0/build")

# 修复libusb符号查找错误：优先使用系统libusb而不是MVS的旧版本
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH FALSE)
set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
set(CMAKE_INSTALL_RPATH "/usr/lib/aarch64-linux-gnu:${CMAKE_INSTALL_RPATH}")

find_package(catkin REQUIRED COMPONENTS
  tf
  tf2_geometry_msgs
  tf2_ros
  roscpp
  rospy
  # pcl library
  pcl_conversions
  # msgs
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  message_generation
  visualization_msgs
)

find_package(OpenMP REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters kdtree search registration features segmentation)
find_package(OpenCV REQUIRED QUIET)
find_package(GTSAM REQUIRED QUIET)
find_package(Boost REQUIRED COMPONENTS timer)

add_message_files(
  DIRECTORY msg
  FILES
  cloud_info.msg
  DynamicTrack.msg
  DynamicTrackArray.msg
  ControlCommand.msg
  SegmentedObjectState.msg
  SegmentedObjectStateArray.msg
  WarningState.msg
)

add_service_files(
  DIRECTORY srv
  FILES
  save_map.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
  nav_msgs
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
  DEPENDS PCL GTSAM

  CATKIN_DEPENDS
  std_msgs
  nav_msgs
  geometry_msgs
  sensor_msgs
  message_runtime
  message_generation
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
)

# include directories
include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
	${GTSAM_INCLUDE_DIR}
)

# link directories
link_directories(
	include
	${PCL_LIBRARY_DIRS}
  ${OpenCV_LIBRARY_DIRS}
  ${GTSAM_LIBRARY_DIRS}
)

###########
## Build ##
###########

# Range Image Projection
add_executable(${PROJECT_NAME}_imageProjection src/imageProjection.cpp)
add_dependencies(${PROJECT_NAME}_imageProjection ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME}_imageProjection ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})

# Feature Association
add_executable(${PROJECT_NAME}_featureExtraction src/featureExtraction.cpp)
add_dependencies(${PROJECT_NAME}_featureExtraction ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME}_featureExtraction ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})

# Mapping Optimization
add_executable(${PROJECT_NAME}_mapOptmization src/mapOptmization.cpp)
add_dependencies(${PROJECT_NAME}_mapOptmization ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_compile_options(${PROJECT_NAME}_mapOptmization PRIVATE ${OpenMP_CXX_FLAGS})
target_link_libraries(${PROJECT_NAME}_mapOptmization Boost::timer ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} ${OpenMP_CXX_FLAGS} gtsam)

# IMU Preintegration
add_executable(${PROJECT_NAME}_imuPreintegration src/imuPreintegration.cpp)
target_link_libraries(${PROJECT_NAME}_imuPreintegration Boost::timer ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} gtsam)

# Dynamic cluster tracker (ID + velocity + prediction markers)
add_executable(${PROJECT_NAME}_dynamicTracker src/dynamicTracker.cpp)
add_dependencies(${PROJECT_NAME}_dynamicTracker ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME}_dynamicTracker ${catkin_LIBRARIES} ${PCL_LIBRARIES})

# Cylindrical monitored-object source (robot-following synthetic target)
add_executable(${PROJECT_NAME}_cylinderObjectSource src/cylinderObjectSource.cpp)
add_dependencies(${PROJECT_NAME}_cylinderObjectSource ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME}_cylinderObjectSource ${catkin_LIBRARIES} ${PCL_LIBRARIES})

# Visual monitored-object selector (publish one shared target for warning/cylinder consumers)
add_executable(${PROJECT_NAME}_visualObjectSelector src/visualObjectSelector.cpp)
add_dependencies(${PROJECT_NAME}_visualObjectSelector ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME}_visualObjectSelector ${catkin_LIBRARIES})

# Static cloud self-filter (remove monitored object occupancy from warning static cloud)
add_executable(${PROJECT_NAME}_staticCloudSelfFilter src/staticCloudSelfFilter.cpp)
add_dependencies(${PROJECT_NAME}_staticCloudSelfFilter ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME}_staticCloudSelfFilter ${catkin_LIBRARIES} ${PCL_LIBRARIES})

# Early warning evaluator (static clearance + dynamic trend fusion)
add_executable(${PROJECT_NAME}_warningEvaluator src/warningEvaluator.cpp)
add_dependencies(${PROJECT_NAME}_warningEvaluator ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME}_warningEvaluator ${catkin_LIBRARIES} ${PCL_LIBRARIES})

# Warning-to-control command bridge (prototype control interface)
add_executable(${PROJECT_NAME}_warningController src/warningController.cpp)
add_dependencies(${PROJECT_NAME}_warningController ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME}_warningController ${catkin_LIBRARIES})
