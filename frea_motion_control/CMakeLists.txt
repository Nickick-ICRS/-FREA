cmake_minimum_required(VERSION 3.0.2)
project(frea_motion_control)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  rospy
  sensor_msgs
  tf2_ros
  tf_conversions
  eigen_conversions
)

find_package(ct_core REQUIRED)
find_package(ct_rbd REQUIRED)
find_package(ct_optcon REQUIRED)

catkin_python_setup()

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(base_link_tf_publisher
    src/base_link_tf_publisher.cpp
    src/frea_motion_control/base_link_tf_calculator.cpp)

target_link_libraries(base_link_tf_publisher
    ${catkin_LIBRARIES})

add_executable(nlmpc
    src/nlmpc.cpp)

target_link_libraries(nlmpc
    ${catkin_LIBRARIES})
