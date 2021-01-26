#ifndef __ROBOT_HPP__
#define __ROBOT_HPP__

#include <ros/ros.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

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
     */
    Robot(const dart::dynamics::SkeletonPtr &skele);

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
     */
    void write();

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
     * @brief Stores joint index for each joint we care about
     */
    std::vector<size_t> jnt_idx_arr_;

    /**
     * @brief What control type are we using for this joint?
     */
    std::vector<ControlType> jnt_ctrl_type_;

    /**
     * @brief ids of any joints to ignore when reading/writing
     */
    std::vector<size_t> ignore_jnts_;
};

#endif // __ROBOT_HPP__
