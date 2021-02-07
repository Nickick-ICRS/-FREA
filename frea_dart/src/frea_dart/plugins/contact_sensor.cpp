#include "frea_dart/plugins/contact_sensor.hpp"

ContactSensor::ContactSensor(
    const dart::simulation::WorldPtr &world,
    const dart::dynamics::SkeletonPtr &robot, std::vector<size_t> body_ids,
    double update_rate, std::string topic_name)

    :world_(world), robot_(robot), body_ids_(body_ids),
     update_period_(1.0/update_rate)
{
    contacts_pub_ = nh_.advertise<frea_dart_msgs::Contact>(topic_name, 1);
    time_since_last_update_ = 0;
}

ContactSensor::~ContactSensor() {
    // dtor
}

void ContactSensor::update(double dt, bool reset) {
    time_since_last_update_ += dt;

    if(time_since_last_update_ >= update_period_) {
        time_since_last_update_ = 0;
        auto contacts = getContacts();
        publishContacts(contacts);
    }

    if(reset) {
        time_since_last_update_ = 0;
    }
}

std::vector<frea_dart_msgs::Contact> ContactSensor::getContacts() {
    std::vector<frea_dart_msgs::Contact> filtered_contacts;

    auto contacts = world_->getLastCollisionResult().getContacts();

    for(const auto &c: contacts) {
        auto sn1 = c.collisionObject1->getShapeFrame()->asShapeNode();
        auto sn2 = c.collisionObject2->getShapeFrame()->asShapeNode();
        
        if(!sn1 || !sn2)
            continue;
        auto bn1 = sn1->getBodyNodePtr();
        auto bn2 = sn2->getBodyNodePtr();

        bool keep = false;
        for(const auto &id : body_ids_) {
            auto bn0 = robot_->getBodyNode(id);
            if(bn0 == bn1 || bn0 == bn2) {
                keep = true;
                break;
            }
        }

        if(keep) {
            frea_dart_msgs::Contact msg;
            msg.object1 = bn1->getName();
            msg.object2 = bn2->getName();
            filtered_contacts.push_back(msg);
        }
    }

    return filtered_contacts;
}

void ContactSensor::publishContacts(
    const std::vector<frea_dart_msgs::Contact> &contacts)
{
    for(const auto &msg : contacts) {
        contacts_pub_.publish(msg);
    }
}
