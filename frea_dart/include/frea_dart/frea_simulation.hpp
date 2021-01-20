#ifndef __FREA_SIMULATION_HPP__
#define __FREA_SIMULATION_HPP__

#include <ros/ros.h>

#include <dart/dart.hpp>
#include <dart/gui/osg/osg.hpp>

template<class T>
void loadParam(const std::string &name, T &val);

class FreaSimulation {
public:
    /**
     * @brief Constructor
     */
    FreaSimulation();

    /**
     * @brief Destructor
     */
    ~FreaSimulation();

    /**
     * @brief Steps the simulation forward by one timestep
     */
    void step();

    /**
     * @brief Resets the simulation, inc. time and model positions...
     */
    void reset();

    /**
     * @brief Runs step and render in a loop
     */
    void run();

    /**
     * @brief Loads a skeleton from the ROS parameter server
     */
    void load_skeleton_param(
        const std::string robot_description="/robot_description");

    /**
     * @brief Loads a skeleton from a file
     */
    void load_skeleton_file(const std::string filepath);

private:
    /**
     * @brief Updates the window display periodically
     *
     * @details This is called every step, and counts how often it should
     *          be updated as per timesteps_per_frame_
     */
    void render();

    /**
     * @brief Sets up the rendering window
     */
    void setup_window();

    /**
     * @brief Loads the world from ROS parameter configuration
     */
    void load_world();

    ros::NodeHandle nh_;

    dart::simulation::WorldPtr world_;
    // Dart doesn't take a shared ptr :(
    dart::gui::osg::WorldNode *world_node_;
    std::unique_ptr<dart::gui::osg::Viewer> viewer_;

    unsigned int timesteps_per_frame_;
    unsigned long long timestep_;
    bool render_;
};

#endif // __FREA_SIMULATION_HPP__
