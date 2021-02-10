#ifndef __ROBOT_HPP__
#define __ROBOT_HPP__

#include <ros/ros.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <urdf/model.h>

#include <dart/dart.hpp>

enum class ControlType {
    POSITION,
    VELOCITY,
    EFFORT,
};

std::ostream& operator<<(std::ostream &os, const ControlType &type);

class Robot : public hardware_interface::RobotHW {
public:
    /**
     * @brief Constructor
     *
     * @param skele The skeleton of the robot
     *
     * @param urdf The URDF model of the robot
     */
    Robot(
        const dart::dynamics::SkeletonPtr &skele,
        const std::shared_ptr<urdf::Model> &urdf);

    /**
     * @brief Destructor
     */
    ~Robot();

    /**
     * @brief Read the joint values from the simulation
     */
    void read();

    /**
     * @brief Write joint commands to the simulation
     *
     * @param period The time duration since the last update
     */
    void write(ros::Duration period);

private:
    /**
     * @brief The skeleton we operate on
     */
    dart::dynamics::SkeletonPtr skele_;

    /**
     * @brief Joint State Interface (see ROS Control)
     */
    hardware_interface::JointStateInterface jnt_state_interface_;

    /**
     * @brief Position Joint Interface (see ROS Control)
     */
    hardware_interface::PositionJointInterface jnt_pos_interface_;

    /**
     * @brief Velocity Joint Interface (see ROS Control)
     */
    hardware_interface::VelocityJointInterface jnt_vel_interface_;

    /**
     * @brief Effort Joint State Interface (see ROS Control)
     */
    hardware_interface::EffortJointInterface jnt_eff_interface_;

    /**
     * @brief Position Joint Limits Interface
     */
    joint_limits_interface::PositionJointSaturationInterface
        jnt_pos_limits_interface_;

    /**
     * @brief Veloctity Joint Limits Interface
     */
    joint_limits_interface::VelocityJointSaturationInterface
        jnt_vel_limits_interface_;

    /**
     * @brief Effort Joint Limits Interface
     */
    joint_limits_interface::EffortJointSaturationInterface
        jnt_eff_limits_interface_;

    /**
     * @brief Command values (see ROS control)
     */
    std::shared_ptr<double[]> cmd_;

    /**
     * @brief Position values (see ROS control)
     */
    std::shared_ptr<double[]> pos_;

    /**
     * @brief Velocity values (see ROS control)
     */
    std::shared_ptr<double[]> vel_;

    /**
     * @brief Effort values (see ROS control)
     */
    std::shared_ptr<double[]> eff_;

    /**
     * @brief What control type are we using for this joint?
     */
    std::shared_ptr<ControlType[]> jnt_ctrl_type_;

    /**
     * @brief Stores joint index for each joint we care about
     */
    std::vector<size_t> jnt_idx_arr_;

    /**
     * @brief ids of any joints to ignore when reading/writing
     */
    std::vector<size_t> ignore_jnts_;
};

#endif // __ROBOT_HPP__
