# FREA Bringup

This is the main entry point for FREA. The launch files within the package and their respective parameters are below:

## frea.launch

The main launch file. This launches everything.

### Parameters
 - hardware (default true) - When set to true this causes hardware-specific nodes to be spun up, such as the hardware interface. When set to false this causes simulation-specific nodes to be spun up as a replacement (see frea_gazebo).

# controllers.launch

Launches the ROS Control controller spawner, and runs the robot state publisher and base_link_tf_publisher nodes.

These nodes allow joints to receive commands, and publish the value of those joints.

The launch file expects a controller configuration file to be loaded, placing the controllers in the namespace `/frea/controllers/<controller_type>/<controller_name>` either by the simulation or hardware launch files.

E.g. `/frea/controllers/position/left_ear_controller`
