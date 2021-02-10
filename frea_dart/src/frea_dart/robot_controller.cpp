#include "frea_dart/robot_controller.hpp"

RobotController::RobotController(
    const dart::dynamics::SkeletonPtr &skele,
    const std::shared_ptr<urdf::Model> &urdf)
{
    robot_.reset(new Robot(skele, urdf));

    // Set up controller manager with its own AsyncSpinner
    nh_.setCallbackQueue(&queue_);
    spinner_.reset(new ros::AsyncSpinner(1, &queue_));
    spinner_->start();
    cm_.reset(new controller_manager::ControllerManager(robot_.get(), nh_));
}

RobotController::~RobotController() {
    // Clean up cm_ first as it has a raw pointer to robot
    cm_.reset();
}


void RobotController::update(
    const ros::Time &time, const ros::Duration &period,
    bool reset_controllers)
{
    robot_->read();
    cm_->update(time, period, reset_controllers);
    robot_->write(period);
}
