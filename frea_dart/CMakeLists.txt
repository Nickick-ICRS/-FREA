cmake_minimum_required(VERSION 3.0.2)
project(frea_dart)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
    frea_msgs
    roscpp
    rospy
    urdf
    controller_manager
    hardware_interface
    eigen_conversions
)
find_package(DART 6.8.2 REQUIRED COMPONENTS
    gui
    gui-osg
    utils-urdf
    collision-bullet)
find_package(Eigen3 REQUIRED)

catkin_package(
    CATKIN_DEPENDS frea_msgs roscpp rospy controller_manager hardware_interface eigen_conversions
    DEPENDS DART
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_plugins
    src/frea_dart/plugins/plugin_manager.cpp
    src/frea_dart/plugins/adjustable_weight.cpp
    src/frea_dart/plugins/contact_sensor.cpp
    src/frea_dart/plugins/imu_sensor.cpp
)
target_link_libraries(${PROJECT_NAME}_plugins
    ${catkin_LIBRARIES} ${DART_LIBRARIES} ${OPENSCENEGRAPH_LIBRARIES})

add_library(${PROJECT_NAME}_frea_simulation
    src/frea_dart/robot.cpp
    src/frea_dart/robot_controller.cpp
    src/frea_dart/pid.cpp
    src/frea_dart/frea_simulation.cpp
)
target_link_libraries(${PROJECT_NAME}_frea_simulation
    ${PROJECT_NAME}_plugins
    ${catkin_LIBRARIES} ${DART_LIBRARIES} ${OPENSCENEGRAPH_LIBRARIES})

add_executable(frea_dart
    src/frea_dart.cpp
)
target_link_libraries(frea_dart
    ${PROJECT_NAME}_frea_simulation)
