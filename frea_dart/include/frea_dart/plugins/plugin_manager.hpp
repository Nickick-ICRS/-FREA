#ifndef __PLUGIN_MANAGER_HPP__
#define __PLUGIN_MANAGER_HPP__

#include "frea_dart/plugins/dart_plugin.hpp"

#include <dart/dart.hpp>

class PluginManager {
public:
    /**
     * @brief Constructor
     */
    PluginManager();

    /**
     * @brief Destructor
     */
    ~PluginManager();

    /**
     * @brief Loads the plugins that we want for a robot
     *
     * @param world The world that the robot is in
     *
     * @param robot The robot
     */
    void loadPlugins(
        const dart::simulation::WorldPtr &world,
        const dart::dynamics::SkeletonPtr &robot);

    /**
     * @brief Updates the plugins, called every simulation step
     *
     * @param dt The time (in s) since the previous update call
     *
     * @param reset Whether the plugins should be reset
     */
    void update(double dt, bool reset);

private:
    std::vector<std::unique_ptr<DartPlugin>> plugins_;
};

#endif // __PLUGIN_MANAGER_HPP__
