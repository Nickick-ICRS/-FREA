cmake_minimum_required(VERSION 2.8.3)
project(frea_gazebo)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    gazebo_plugins
    gazebo_ros
    roscpp
)

find_package(gazebo REQUIRED)

catkin_package(
    CATKIN_DEPENDS gazebo_ros gazebo_plugins roscpp
)

include_directories(
    plugins/include
    ${catkin_INCLUDE_DIRS}
    ${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEOBO_CXX_FLAGS}")

add_library(frea_adjustable_weight SHARED plugins/src/adjustable_weight.cpp)
target_link_libraries(frea_adjustable_weight
    ${catkin_LIBRARIES}
    ${GAZEBO_LIBRARIES})
