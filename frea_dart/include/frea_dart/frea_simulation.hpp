#ifndef __FREA_SIMULATION_HPP__
#define __FREA_SIMULATION_HPP__

#include <ros/ros.h>

#include <dart/dart.hpp>
#include <dart/gui/osg/osg.hpp>

#include "frea_dart/robot_controller.hpp"

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
     *
     * @param robot_description The param from which to load the skeleton
     *
     * @param ctrl If true then we spawn ROS Control stuff for this skele
     */
    void loadSkeletonParam(
        const std::string robot_description="/robot_description",
        bool ctrl=true);

    /**
     * @brief Loads a skeleton from a file
     *
     * @param filepath The file from which to load the skeleton
     *
     * @param ctrl If true then we spawn ROS Control stuff for this skele
     */
    void loadSkeletonFile(const std::string filepath, bool ctrl=true);

    /**
     * @brief Sets the pose of a skeleton
     *
     * @param name The name of the skeleton
     *
     * @param pose The pose (in the world frame) to set
     *
     * @returns true if it succeeded else false
     */
    bool setSkeletonPose(
        const std::string name, const Eigen::Affine3d &pose);

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
    void setupWindow();

    /**
     * @brief Set up ROS pub/subs
     */
    void setupPubSubs();

    /**
     * @brief Loads the world from ROS parameter configuration
     */
    void loadWorld();
    
    /**
     * @brief Checks if the simulation window has been closed
     *
     * @returns true if the window has been closed. false if it is open, or
     *          if we are not rendering
     */
    bool windowClosed();

    /**
     * @brief Publishes the current simulation time
     */
    void publishTime();

    /**
     * @brief Handles ROS callbacks forever... Run in a parallel thread
     */
    void spinRos();

    ros::NodeHandle nh_;

    dart::simulation::WorldPtr world_;
    // Dart won't let me use unique_ptr :(
    dart::gui::osg::WorldNode *world_node_;
    std::unique_ptr<dart::gui::osg::Viewer> viewer_;

    std::unique_ptr<RobotController> robot_ctrl_;

    ros::Publisher clock_pub_;

    unsigned int timesteps_per_frame_;
    unsigned long long timestep_;
    bool render_;
    ros::Time time_;

    std::unique_ptr<ros::AsyncSpinner> spinner_;
};

#endif // __FREA_SIMULATION_HPP__
