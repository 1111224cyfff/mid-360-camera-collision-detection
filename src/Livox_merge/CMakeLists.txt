cmake_minimum_required(VERSION 3.0.2)
project(livox_merge)

## Compile as C++14, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  pcl_ros
  pcl_conversions
  livox_ros_driver2
  tf2
  tf2_ros
)

## System dependencies are found with CMake's conventions
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenMP REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")

# Ensure variables expected by catkin_package() are defined when using
# config-mode Find (Eigen3Config) which provides the target Eigen3::Eigen
if(TARGET Eigen3::Eigen)
  get_target_property(_eigen_includes Eigen3::Eigen INTERFACE_INCLUDE_DIRECTORIES)
  if(_eigen_includes)
    set(Eigen3_INCLUDE_DIRS ${_eigen_includes})
    set(Eigen3_LIBRARIES Eigen3::Eigen)
    # backward-compatible variable sometimes used in projects
    set(EIGEN3_INCLUDE_DIRS ${_eigen_includes})
  endif()
endif()


###################################
## catkin specific configuration ##
###################################
catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS 
    roscpp 
    rospy 
    std_msgs 
    sensor_msgs 
    geometry_msgs 
    pcl_ros 
    pcl_conversions 
    livox_ros_driver2
    tf2
    tf2_ros
  DEPENDS 
    PCL
    Eigen3
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

## Add executable
add_executable(merge_lidar_node src/MergeLidar.cpp)

## One-time IMU-based pointcloud leveling node
add_executable(merged_pointcloud_leveler_node src/merged_pointcloud_leveler_node.cpp)

## Add cmake target dependencies of the executable
add_dependencies(merge_lidar_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_dependencies(merged_pointcloud_leveler_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(merge_lidar_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  OpenMP::OpenMP_CXX
)

target_link_libraries(merged_pointcloud_leveler_node
  ${catkin_LIBRARIES}
)

## Compiler-specific options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  target_compile_options(merge_lidar_node PRIVATE -Wall -Wextra -O3)
endif()

#############
## Install ##
#############

## Mark executable scripts (Python etc.) for installation
catkin_install_python(PROGRAMS
  scripts/test_imu_time_order.py
  scripts/test_fusion_system.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark executables for installation
install(TARGETS merge_lidar_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS merged_pointcloud_leveler_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY
  launch/
  config/
  rviz/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
