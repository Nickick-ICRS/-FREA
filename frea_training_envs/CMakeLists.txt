cmake_minimum_required(VERSION 3.0.2)
project(frea_training_envs)

find_package(catkin REQUIRED COMPONENTS
  openai_ros
  rospy
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS openai_ros rospy
)
