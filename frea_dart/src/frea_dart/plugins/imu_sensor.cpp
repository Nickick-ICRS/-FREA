#include "frea_dart/plugins/imu_sensor.hpp"

#include <eigen_conversions/eigen_msg.h>

ImuSensor::ImuSensor(
    const dart::dynamics::SkeletonPtr robot, std::string body_name,
    std::string topic_name, double update_rate, double gaussian_noise)
    
    :gen_(std::random_device{}()), noise_(0, gaussian_noise),
     update_period_(1.0/update_rate), variance_(pow(gaussian_noise, 2))
{
    body_ = robot->getBodyNode(body_name);
    if(!body_) {
        ROS_ERROR_STREAM(
            body_name << " does not exist in robot " << robot->getName()
            << "!");
        return;
    }

    imu_pub_ = nh_.advertise<sensor_msgs::Imu>(topic_name, 1);
    time_since_last_update_ = 0;
}

ImuSensor::~ImuSensor() {
    // dtor
}

void ImuSensor::update(double dt, bool reset) {
    if(!body_)
        return;

    time_since_last_update_ += dt;

    if(time_since_last_update_ >= update_period_) {
        time_since_last_update_ = 0;

        sensor_msgs::Imu imu = calculateImuData();
        imu_pub_.publish(imu);
    }

    if(reset) {
        time_since_last_update_ = 0;
    }
}

sensor_msgs::Imu ImuSensor::calculateImuData() {
    sensor_msgs::Imu msg;

    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = body_->getName();

    // Grab orientation
    Eigen::Affine3d pose = body_->getWorldTransform();
    // Apply noise in r, p and y
    pose *= Eigen::AngleAxisd(noise_(gen_), Eigen::Vector3d::UnitX());
    pose *= Eigen::AngleAxisd(noise_(gen_), Eigen::Vector3d::UnitY());
    pose *= Eigen::AngleAxisd(noise_(gen_), Eigen::Vector3d::UnitZ());
    geometry_msgs::Pose ros_pose;
    tf::poseEigenToMsg(pose, ros_pose);
    msg.orientation = ros_pose.orientation;

    msg.orientation_covariance[0] = variance_;
    msg.orientation_covariance[4] = variance_;
    msg.orientation_covariance[8] = variance_;

    // Grab angular velocities
    const Eigen::Vector6d &vels = body_->getSpatialVelocity();
    // 0-2 are rpy, 3-5 are XYZ
    msg.angular_velocity.x = vels[0] + noise_(gen_);
    msg.angular_velocity.y = vels[1] + noise_(gen_);
    msg.angular_velocity.z = vels[2] + noise_(gen_);

    msg.angular_velocity_covariance[0] = variance_;
    msg.angular_velocity_covariance[4] = variance_;
    msg.angular_velocity_covariance[8] = variance_;

    // Grab linear accelerations
    const Eigen::Vector6d &accs = body_->getSpatialAcceleration();
    // 0-2 are rpy, 3-5 are XYZ
    msg.linear_acceleration.x = accs[3] + noise_(gen_);
    msg.linear_acceleration.y = accs[4] + noise_(gen_);
    msg.linear_acceleration.z = accs[5] + noise_(gen_);

    msg.linear_acceleration_covariance[0] = variance_;
    msg.linear_acceleration_covariance[4] = variance_;
    msg.linear_acceleration_covariance[8] = variance_;

    return msg;
}
