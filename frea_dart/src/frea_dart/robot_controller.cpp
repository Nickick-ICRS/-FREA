#include "frea_dart/robot_controller.hpp"

RobotController::RobotController(const dart::dynamics::SkeletonPtr &skele)
{
    robot_.reset(new Robot(skele));
    cm_.reset(new controller_manager::ControllerManager(robot_.get()));
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
    robot_->write();
}
