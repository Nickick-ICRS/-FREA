#include "frea_motion_control/base_link_tf_calculator.hpp"

int main(int argc, char **argv) {
    ros::init(argc, argv, "base_link_tf_publisher");

    frea_motion_control::BaseLinkTfCalculator calc;
    calc.spin();
}
