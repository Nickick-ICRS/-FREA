#ifndef __ADJUSTABLE_WEIGHT_HPP__
#define __ADJUSTABLE_WEIGHT_HPP__

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <std_msgs/Float32.h>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

namespace gazebo {

/**
 * @brief Helper function to load parameters from SDF, including defaults
 */
template<typename T>
void loadParam(
    std::string param_name, T &param, sdf::ElementPtr &sdf);

/**
 * @brief Plugin to simulate an adjustable weight on a link of the robot.
 *
 * @details This allows us to train a controller which is capable of
 *          balancing FREA with an unknown weight attached to the head or
 *          tail (or both, with two instances of the plugin)
 */
class AdjustableWeight :public ModelPlugin {
public:
    /**
     * @brief Constructor
     */
    AdjustableWeight();

    /**
     * @brief Destructor
     */
    ~AdjustableWeight();

    /**
     * @brief Setup function called when Gazebo loads the plugin
     *
     * @param model Pointer to the FREA model
     *
     * @param sdf Pointer to the SDF description of FREA
     */
    void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override;

private:
    /**
     * @brief Callback to adjust the mass and inertia etc. of the
     *        imaginary object
     *
     * @param msg The ROS message containing the new mass (Kg)
     */
    void changeWeightCallback(const std_msgs::Float32ConstPtr &msg);

    /**
     * @brief Thread to run ROS callbacks when Gazebo is e.g. paused
     */
    void ROSQueueThread();

    /**
     * @brief Update function called every frame by Gazebo
     */
    void OnUpdate();

    /**
     * @brief NodeHandle
     *
     * @details We create this in the Load function rather than the
     *          constructor, so it needs to be a pointer.
     */
    std::shared_ptr<ros::NodeHandle> nh_;

    /**
     * @brief Subscriber to for the changeWeightCallback
     */
    ros::Subscriber weight_sub_;

    /**
     * @brief Queue to handle ROS callbacks
     */
    ros::CallbackQueue ros_queue_;

    /**
     * @brief Thread to handle ROS queue
     */
    std::thread callback_queue_thread_;

    /**
     * @brief The link we adjust the mass of
     */
    physics::LinkPtr link_;

    /**
     * @brief The force applied every frame to simulate the mass
     */
    ignition::math::Vector3d force_;

    /**
     * @brief The position the force is applied at
     */
    ignition::math::Vector3d position_;

    /**
     * @brief Connection to the update event
     */
    event::ConnectionPtr update_connection_;
};

}; // ns gazebo

#endif // __ADJUSTABLE_WEIGHT_HPP__
