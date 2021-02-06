#include <ros/ros.h>

#include "frea_dart/frea_simulation.hpp"

int main(int argc, char **argv) {
    ros::init(argc, argv, "frea_simulator");

    // Robot URDF description
    std::string robot_desc;
    // Robot starting position in simulation
    double X, Y, Z;
    ros::param::param<std::string>(
        "/frea_dart/robot/parameter", robot_desc, "/robot_description");
    ros::param::param<double>("/frea_dart/robot/X", X, 0);
    ros::param::param<double>("/frea_dart/robot/Y", Y, 0);
    ros::param::param<double>("/frea_dart/robot/Z", Z, 0);

    FreaSimulation sim;

    sim.loadSkeletonParam();

    Eigen::Affine3d af;
    af = Eigen::Translation3d(X, Y, Z);
    sim.setSkeletonPose("frea", af);

    sim.run();
}
