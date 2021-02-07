cmake_minimum_required(VERSION 3.0.2)
project(frea_dart_msgs)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  std_msgs
  sensor_msgs
  message_generation
)

# Generate messages in the 'msg' folder
add_message_files(
  FILES
  Contact.msg
)

# Generate services in the 'srv' folder
add_service_files(
  FILES
  GetBodyState.srv
  GetRobotState.srv
  SetRobotState.srv
  SpawnRobot.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS std_msgs geometry_msgs sensor_msgs
)
