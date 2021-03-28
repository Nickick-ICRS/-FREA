cmake_minimum_required(VERSION 3.0.2)
project(frea_hardware)

find_package(catkin REQUIRED COMPONENTS
    rospy
    frea_msgs
)

catkin_python_setup()

catkin_package(
    CATKIN_DEPENDS rospy frea_msgs
)

catkin_install_python(PROGRAMS
    nodes/frea_head
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
