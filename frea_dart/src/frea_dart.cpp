#include <ros/ros.h>

#include "frea_dart/frea_simulation.hpp"

int main(int argc, char **argv) {
    ros::init(argc, argv, "frea_simulator");

    FreaSimulation sim;

    sim.loadSkeletonParam();

    Eigen::Affine3d af;
    af = Eigen::Translation3d(0, 0, 1);
    sim.setSkeletonPose("frea", af);

    sim.run();
}
