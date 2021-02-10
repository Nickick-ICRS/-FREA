#ifndef __ROBOT_CONTROLLER_HPP__
#define __ROBOT_CONTROLLER_HPP__

#include "frea_dart/robot.hpp"

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <controller_manager/controller_manager.h>
#include <urdf/model.h>

#include <dart/dart.hpp>

// This is the class you care about
class RobotController {
public:
    /**
     * @brief Constructor
     *
     * @param skele The skeleton of the robot to control
     *
     * @param urdf The urdf of the robot
     */
    RobotController(
        const dart::dynamics::SkeletonPtr &skele,
        const std::shared_ptr<urdf::Model> &urdf);

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

    /**
     * @brief Nodehandle for our personal callback queue
     */
    ros::NodeHandle nh_;

    /**
     * @brief Seperate callback queue for the controller manager
     */
    ros::CallbackQueue queue_;

    /**
     * @brief Spinner for our personal callback queue
     */
    std::shared_ptr<ros::AsyncSpinner> spinner_;
};

#endif // __ROBOT_CONTROLLER_HPP__
