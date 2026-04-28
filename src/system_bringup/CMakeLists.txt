cmake_minimum_required(VERSION 3.0.2)
project(system_bringup)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
        FILES_MATCHING PATTERN "*.launch"
)

install(DIRECTORY rviz/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
        FILES_MATCHING PATTERN "*.rviz"
)

catkin_install_python(PROGRAMS
        scripts/adjust_bag_exposure.py
        scripts/export_bag_prediction_wrapper.py
        scripts/img_exetract_with_time.py
        scripts/topic_image_timestamp_saver.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS
        scripts/ptp_time_sync.sh
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

