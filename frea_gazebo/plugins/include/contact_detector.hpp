#ifndef __CONTACT_DETECTOR_HPP__
#define __CONTACT_DETECTOR_HPP__

#include <ros/ros.h>
#include <frea_msgs/Contact.h>

#include <gazebo/gazebo.hh>
#include <gazebo/transport/transport.hh>

namespace gazebo {

/**
 * @brief Plugin to detect contacts between FREA and other objects
 *
 * @details Publishes whether FREA is in contact with an external object.
 *          Links from which contacts should be ignored (e.g. wheels) can
 *          be specified in SDF. This is primarily used to train FREA to
 *          not bump into objects, or drag its tail.
 */
class ContactDetector :public ModelPlugin {
public:
    /**
     * @brief Constructor
     */
    ContactDetector();

    /**
     * @brief Destructor
     */
    virtual ~ContactDetector();

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
     * @brief Processes and publishes contacts to ROS
     *
     * @details Drops contacts if they involve one of the ignored links
     *
     * @param contacts The contacts to be processed
     */
    void publishContacts(
        const boost::shared_ptr<const msgs::Contacts> &contacts);

    /**
     * @brief Unscopes a gazebo collision, by splitting it about '::'
     *
     * @param s The collision name to be unscoped
     *
     * @returns The unscoped collision
     */
    std::string unscopeCollision(std::string s);

    /**
     * @brief Node to connect to gazebo topics
     */
    gazebo::transport::NodePtr gz_node_;

    /**
     * @brief Subscriber to gazebo contact topic
     */
    gazebo::transport::SubscriberPtr gz_contact_sub_;

    /**
     * @brief List of links which we ignore contacts from
     */
    std::vector<std::string> ignored_links_;

    /**
     * @brief Pointer to node handle
     */
    std::shared_ptr<ros::NodeHandle> nh_;

    /**
     * @brief Publisher for detected contacts
     */
    ros::Publisher contact_pub_;
};

} // ns gazebo

#endif // __CONTACT_DETECTOR_HPP__
