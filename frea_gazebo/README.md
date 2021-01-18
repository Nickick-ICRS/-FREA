# FREA Gazebo

This package contains the launch file to launch the gazebo simulation environment as well as plugins and world files to go with it.

## Launch

There are currently no launch parameters.

## Plugins

### AdjustableWeight

This plugin allows for a runtime-adjustable weight to be added to any link in the robot. This is implemented by applying a constant force equal to `-9.8*mass` to the link. It has the following parameters:
 - **topicName**: The topic to subscribe to for weight change messages (std_msgs/Float32)
 - **linkName**: The link to apply the changes to
 - **position**: The position relative to the link at which to apply the mass

Plugin filename: "libfrea_adjustable_weight.so"

### ContactDetector

This plugin detects contacts between FREA and her environment, and publishes them to ROS. It drops certain messages if requested.
 - **topicName**: The topic on which to publish contacts (frea_msgs/Contact)
 - **ignoredCollisionN**: The Nth collision to ignore contacts from, e.g. `<ignoredCollision0>left_wheel_link_collision</ignoredCollision0>`

Plugin filename: "libfrea_contact_detector.so"

## World Files

There are currently no world files.

## Config

### ros_controllers.yaml

This config file contains the PID values for the gazebo joint controllers and the required controller setup to enable ROS control to interface with them.
