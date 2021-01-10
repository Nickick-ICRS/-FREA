#include "adjustable_weight.hpp"

#include <string>

#include <ignition/math.hh>

namespace gazebo {

GZ_REGISTER_MODEL_PLUGIN(AdjustableWeight)

template<typename T>
void loadParam(std::string param_name, T &param, sdf::ElementPtr &sdf)
{
    if(sdf->HasElement(param_name)) {
        param = sdf->Get<T>(param_name);
    }
    else {
        ROS_WARN_STREAM(
            "AdjustableWeight failed to get SDF parameter '" << param_name
            << "'. Defaulting to " << param);
    }
}


AdjustableWeight::AdjustableWeight() {
    // ctor
}

AdjustableWeight::~AdjustableWeight() {
    nh_->shutdown();
    ros_queue_.clear();
    ros_queue_.disable();
    callback_queue_thread_.join();
}

void AdjustableWeight::Load(physics::ModelPtr model, sdf::ElementPtr sdf) {
    // Crash if ROS isn't running
    if(!ros::isInitialized()) {
        ROS_FATAL_STREAM(
            "A ROS node for Gazebo has not been initialized, unable to "
            << "load plugin. Load the Gazebo system plugin "
            << "'libgazebo_ros_api_plugin.so' in the gazebo_ros package");
        return;
    }

    ROS_INFO("Loading AdjustableWeight plugin");

    std::string topic_name = "/change_weight";
    std::string name;

    loadParam("topicName", topic_name, sdf);
    loadParam("linkName", name, sdf);
    loadParam("position", position_, sdf);

    link_ = model->GetLink(name);
    if(!link_) {
        ROS_FATAL_STREAM("Link '" << name << "' does not exist!");
        return;
    }

    nh_.reset(new ros::NodeHandle());
    nh_->setCallbackQueue(&ros_queue_);
    weight_sub_ = nh_->subscribe<std_msgs::Float32>(
        topic_name, 1, &AdjustableWeight::changeWeightCallback, this);

    callback_queue_thread_ = std::thread(
        std::bind(&AdjustableWeight::ROSQueueThread, this));

    update_connection_ = event::Events::ConnectWorldUpdateBegin(
        std::bind(&AdjustableWeight::OnUpdate, this));

    ROS_INFO("Finished loading AdjustableWeight plugin");
}

void AdjustableWeight::changeWeightCallback(
    const std_msgs::Float32ConstPtr &msg)
{
    force_.Z() = msg->data <= 0 ? 0 : msg->data * -9.8;
}

void AdjustableWeight::ROSQueueThread() {
    const double timeout = 0.04;
    while(nh_->ok()) {
        ros_queue_.callAvailable(ros::WallDuration(timeout));
    }
}

void AdjustableWeight::OnUpdate() {
    link_->AddForceAtRelativePosition(force_, position_);
}

}; // ns gazebo
