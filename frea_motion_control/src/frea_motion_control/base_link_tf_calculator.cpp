#include "frea_motion_control/base_link_tf_calculator.hpp"

#include <numeric>
#include <geometry_msgs/TransformStamped.h>
#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>

namespace frea_motion_control {

BaseLinkTfCalculator::BaseLinkTfCalculator() :nh_("~") {
    // Set parameter defaults
    base_link_ = "base_link";
    chassis_link_ = "chassis_link";
    ma_history_length_ = 10;

    // Load parameters
    if(!nh_.getParam("base_link", base_link_)) {
        ROS_WARN_STREAM(
            "Failed to get param 'base_link'. Defaulting to: "
            << base_link_);
    }
    if(!nh_.getParam("chassis_link", chassis_link_)) {
        ROS_WARN_STREAM(
            "Failed to get param 'chassis_link'. Defaulting to: "
            << chassis_link_);
    }
    if(!nh_.getParam("ma_history_length", ma_history_length_)) {
        ROS_WARN_STREAM(
            "Failed to get param 'ma_history_length'. Defaulting to: "
            << ma_history_length_);
    }
}

BaseLinkTfCalculator::~BaseLinkTfCalculator() {
    // dtor
}

void BaseLinkTfCalculator::spin() {
    // We only buffer 2 because we only care about the most recent readings
    imu_sub_ = nh_.subscribe<sensor_msgs::Imu>(
        "/imu/data", 2, &BaseLinkTfCalculator::imuCallback, this);

    ros::spin();
}

void BaseLinkTfCalculator::imuCallback(const sensor_msgs::ImuConstPtr &msg)
{
    geometry_msgs::TransformStamped tf;
    tf.header.frame_id = base_link_;
    tf.header.stamp = msg->header.stamp;
    tf.child_frame_id = chassis_link_;

    Eigen::Quaterniond quat;
    tf::quaternionMsgToEigen(msg->orientation, quat);

    double pitch = quat.toRotationMatrix().eulerAngles(0, 1, 2)[1];

    double avg_pitch = calculatePitchMA(pitch);

    quat = Eigen::AngleAxisd(avg_pitch, Eigen::Vector3d(0, 1, 0));
    tf::quaternionEigenToMsg(quat, tf.transform.rotation);

    tf2_br_.sendTransform(tf);
}

double BaseLinkTfCalculator::calculatePitchMA(double pitch) {
    pitch_values_.push_front(pitch);
    if(pitch_values_.size() > ma_history_length_) {
        pitch_values_.pop_back();
    }

    double avg = std::accumulate(
        pitch_values_.begin(), pitch_values_.end(), 0.0);
    return avg / pitch_values_.size();
}

}; // ns frea_motion_control
