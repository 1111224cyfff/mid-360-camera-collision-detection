cmake_minimum_required(VERSION 3.0.2)
project(jetson_vision_detect)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  rospy
  sensor_msgs
  std_msgs
)

add_message_files(
  FILES
  TrackedDetection2D.msg
  TrackedDetections2D.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime rospy sensor_msgs std_msgs
)

catkin_install_python(PROGRAMS
  scripts/detect_track_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
