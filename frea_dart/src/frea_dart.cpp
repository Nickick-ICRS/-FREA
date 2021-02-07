#include <ros/ros.h>

#include "frea_dart/frea_simulation.hpp"

int main(int argc, char **argv) {
    ros::init(argc, argv, "frea_simulator");

    FreaSimulation sim;

    sim.run();
}
