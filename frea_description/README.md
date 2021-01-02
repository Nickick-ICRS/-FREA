# FREA Description

This package contains the xacro'ed URDF files for FREA. These are split into four categories:
 - chassis - The xacro files for FREA's lower chassis
 - head - The xacro files for FREA's head, mouth and ears
 - neck - The xacro files for FREA's neck and tail
 - sensors - IMU and Camera sensors

All of these are combined together within the main `frea.urdf.xacro` file, which is what should be loaded into the robot_description in the launch files.
