#ifndef __ROBOT_CONTROLLER_HPP__
#define __ROBOT_CONTROLLER_HPP__

#include "frea_dart/robot.hpp"

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <controller_manager/controller_manager.h>

#include <dart/dart.hpp>

// This is the class you care about
class RobotController {
public:
    /**
     * @brief Constructor
     *
     * @param skele The skeleton of the robot to control
     */
    RobotController(const dart::dynamics::SkeletonPtr &skele);

    /**
     * @brief Destructor
     */
    ~RobotController();

    /**
     * @brief Updates the controllers
     *
     * @param time The current time
     *
     * @param period The change in time since the last call to @ref update
     *
     * @param reset_controller If @c true, then reset all controllers before
     *                         updating
     */
    void update(
        const ros::Time &time, const ros::Duration &period,
        bool reset_controllers=false);

private:
    /**
     * @brief The robot which we are controlling
     */
    std::shared_ptr<Robot> robot_;

    /**
     * @brief The controller manager interface
     */
    std::shared_ptr<controller_manager::ControllerManager> cm_;
};

#endif // __ROBOT_CONTROLLER_HPP__
