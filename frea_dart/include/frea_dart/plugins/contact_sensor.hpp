#ifndef __CONTACT_SENSOR_HPP__
#define __CONTACT_SENSOR_HPP__

#include "frea_dart/plugins/dart_plugin.hpp"

#include <ros/ros.h>
#include <frea_dart_msgs/Contact.h>

#include <dart/dart.hpp>

class ContactSensor :public DartPlugin {
public:
    /**
     * @brief Constructor
     *
     * @param world The world the robot is in
     *
     * @param robot The robot we're connected to
     *
     * @param body_ids Ids of the bodies we listen for contacts on
     *
     * @param update_rate How often (per s) the plugin should be updated
     *
     * @param topic_name The name of the topic on which to publish the
     *                   contacts
     */
    ContactSensor(
        const dart::simulation::WorldPtr &world,
        const dart::dynamics::SkeletonPtr &robot,
        std::vector<size_t> body_ids, double update_rate,
        std::string topic_name);

    /**
     * @brief Destructor
     */
    virtual ~ContactSensor();

    /**
     * @brief Update function, called every timestep
     *
     * @param dt The (simulation) time since the last update (in s)
     *
     * @param reset Whether to reset the sensor
     */
    virtual void update(double dt, bool reset) override;

protected:

    /**
     * @brief Looks for contacts in the simulation between the specified
     *        bodys and other objects
     *
     * @returns Vector of contacts
     */
    std::vector<frea_dart_msgs::Contact> getContacts();
 
    /**
     * @brief Publishes the detected contacts to ROS
     *
     * @param contacts The detected contacts
     */
    void publishContacts(
        const std::vector<frea_dart_msgs::Contact> &contacts);

    ros::NodeHandle nh_;
    ros::Publisher contacts_pub_;

    dart::dynamics::SkeletonPtr robot_;
    dart::simulation::WorldPtr world_;
    std::vector<size_t> body_ids_;

    double time_since_last_update_;
    double update_period_;
};

#endif // __CONTACT_SENSOR_HPP__
