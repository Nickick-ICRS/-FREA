#frea_motion_control

This packages contains the main joint controller node, as well as some convenience nodes for TF.

A description of the nodes and launch files ensues:

## Launch Files

### base_link_tf_publisher

This launch file launches the base_link_tf_publisher node (see below). It does not have any parameters.

## Nodes

### base_link_tf_publisher

This node publishes the transform from the chassis link to the base link of the robot. As the chassis link is not constrained about the rotational y axis (pitch), an additional transform is required to keep the base link flat to the surface.

Currently this transform is calculated via a moving average of the most recent IMU readings. In the future we may incorporate pitch estimation from the head camera via MonoSLAM etc., at which point this may be replaced with a more intelligent EKF or something similar.

Parameters:
 - base_link (string) The name of the base_link frame in TF. Defaults to "base_link"
 - chassis_link (string) The name of the chassis_link frame in TF. Defaults to "chassis_link"
 - ma_history_length (unsigned int) The number of previous readings to incorporate into the moving average calculation of the pitch of the chassis. Defaults to 10.
