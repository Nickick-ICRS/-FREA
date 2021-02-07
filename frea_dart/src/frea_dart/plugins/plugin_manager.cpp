#include "frea_dart/plugins/plugin_manager.hpp"

#include "frea_dart/plugins/adjustable_weight.hpp"
#include "frea_dart/plugins/contact_sensor.hpp"
#include "frea_dart/plugins/imu_sensor.hpp"

PluginManager::PluginManager() {
    // ctor
}

PluginManager::~PluginManager() {
    // dtor
}

void PluginManager::loadPlugins(
    const dart::simulation::WorldPtr &world,
    const dart::dynamics::SkeletonPtr &robot)
{
    if(robot->getName() == "frea") {
        std::vector<size_t> body_ids;
        size_t num_bodies = robot->getNumBodyNodes();
        for(size_t i = 0; i < robot->getNumBodyNodes(); i++) {
            const auto *bn = robot->getBodyNode(i);
            if(bn->getName() == "left_wheel_link" ||
               bn->getName() == "right_wheel_link")
            {
                continue;
            }
            body_ids.emplace_back(i);
        }
        plugins_.emplace_back(new ContactSensor(
            world, robot, body_ids, -1, "/contacts"));

        plugins_.emplace_back(new AdjustableWeight(
            robot, "/adjust_weight/head", "head_link",
            Eigen::Vector3d(0.08, 0, -0.082)));

        plugins_.emplace_back(new AdjustableWeight(
            robot, "/adjust_weight/tail", "lower_tail_link",
            Eigen::Vector3d(0, 0, 0.225)));

        plugins_.emplace_back(new ImuSensor(
            robot, "imu_link", "/imu/data", 100, 0.01));
    }
    else {
        ROS_WARN_STREAM(
            "Only plugins for 'frea' are supported, ignoring robot '"
            << robot->getName() << "'.");
    }
}

void PluginManager::update(double dt, bool reset) {
    for(const auto &plugin : plugins_) {
        plugin->update(dt, reset);
    }
}
