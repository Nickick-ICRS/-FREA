#include "contact_detector.hpp"
#include "load_param.hpp"

#include <sstream>

namespace gazebo {

GZ_REGISTER_MODEL_PLUGIN(ContactDetector)

ContactDetector::ContactDetector() {
    // ctor
}

ContactDetector::~ContactDetector() {
    // dtor
}

void ContactDetector::Load(
    physics::ModelPtr model, sdf::ElementPtr sdf)
{
    if(!ros::isInitialized()) {
        ROS_FATAL_STREAM(
            "A ROS node for Gazebo has not been initialized, unable to "
            << "load plugin. Load the Gazebo system plugin "
            << "'libgazebo_ros_api_plugin.so' in the gazebo_ros package");
        return;
    }

    ROS_INFO("Loading ContactDetector plugin");

    std::string topic_name = "/contacts";

    loadParam("topicName", topic_name, sdf);

    std::string ignored_link = "";
    std::string element_name = "ignoredCollision";
    unsigned int element_num = 0;
    std::stringstream ss;
    while(true) {
        std::string str;
        ss.clear();
        ss << element_num++;
        ss >> str;
        str = element_name+str;
        loadParam(str, ignored_link, sdf);
        if(ignored_link == "") break;
        ignored_links_.push_back(ignored_link);
        ignored_link = "";
    }

    nh_.reset(new ros::NodeHandle());

    contact_pub_ = nh_->advertise<frea_msgs::Contact>(topic_name, 1);

    gz_node_.reset(new transport::Node());
    gz_node_->Init();

    gz_contact_sub_ = gz_node_->Subscribe(
        "~/physics/contacts", &ContactDetector::publishContacts, this);
}

void ContactDetector::publishContacts(
    const boost::shared_ptr<const msgs::Contacts> &contacts)
{
    for(const auto &c : contacts->contact()) {
        // Split c1 to find the un-scoped collision name
        std::string c1 = unscopeCollision(c.collision1()); 
        std::string c2 = unscopeCollision(c.collision2());

        bool skip = false;
        for(const auto &str : ignored_links_) {
            if(str == c1 || str == c2) {
                skip = true;
                break;
            }
        }

        if(skip)
            continue;

        frea_msgs::Contact msg;
        msg.object1 = c1;
        msg.object2 = c2;

        contact_pub_.publish(msg);
    }
}

std::string ContactDetector::unscopeCollision(std::string s) {
    std::string o = s;
    std::string::size_type p = 0;
    std::string::size_type q = 0;
    // Find the positition of the final "::" in the string
    do {
        q += p+2;
        p = s.find("::");
        s = s.substr(p+2);
    }
    while(p != std::string::npos);

    // Split the string
    return o.substr(q-2);
}

} // ns gazebo
