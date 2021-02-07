#ifndef __ADJUSTABLE_WEIGHT_HPP__
#define __ADJUSTABLE_WEIGHT_HPP__

#include "frea_dart/plugins/dart_plugin.hpp"

#include <ros/ros.h>
#include <std_msgs/Float32.h>

#include <dart/dart.hpp>

class AdjustableWeight :public DartPlugin {
public:
    /**
     * @brief Constructor
     *
     * @param robot The robot this plugin attaches to
     *
     * @param topic_name The topic on which to listen for commands
     *
     * @param body_name The name of the body to apply force to
     *
     * @param position The position (relative to the body) to apply force at
     */
    AdjustableWeight(
        const dart::dynamics::SkeletonPtr robot, std::string topic_name,
        std::string body_name, Eigen::Vector3d position);

    /**
     * @bried Destructor
     */
    virtual ~AdjustableWeight();

    /**
     * @brief Update function, called once per simulation step
     *
     * @param dt The time since the last update (in s)
     *
     * @param reset Whether to reset the plugin or not
     */
    virtual void update(double dt, bool reset) override;
protected:

    /**
     * @brief Callback to update the "weight" of the body
     *
     * @param msg The new weight of the body
     */
    void changeWeightCallback(const std_msgs::Float32ConstPtr &msg);

    ros::NodeHandle nh_;
    ros::Subscriber weight_sub_;

    dart::dynamics::SkeletonPtr robot_;
    dart::dynamics::BodyNodePtr body_;
};

#endif // __ADJUSTABLE_WEIGHT_HPP__
