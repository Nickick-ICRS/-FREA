#ifndef __FREA_SIMULATION_HPP__
#define __FREA_SIMULATION_HPP__

#include <ros/ros.h>

#include <std_srvs/Empty.h>
#include <frea_dart_msgs/SpawnRobot.h>
#include <frea_dart_msgs/GetRobotState.h>
#include <frea_dart_msgs/SetRobotState.h>

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
    void reset(bool reset_time=true, bool reset_robots=false);

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
    bool loadSkeletonParam(
        const std::string robot_description="/robot_description",
        bool ctrl=true);

    /**
     * @brief Loads a skeleton from a file
     *
     * @param filepath The file from which to load the skeleton
     *
     * @param ctrl If true then we spawn ROS Control stuff for this skele
     */
    bool loadSkeletonFile(const std::string filepath, bool ctrl=true);

    /**
     * @brief Sets the pose of a skeleton
     *
     * @param name The name of the skeleton
     *
     * @param pose The pose (in the world frame) to set
     *
     * @param set_initial Set the initial state of the robot to be here
     *                    so that when the simulation is reset it returns to
     *                    this pose
     *
     * @returns true if it succeeded else false
     */
    bool setSkeletonPose(
        const std::string name, const Eigen::Affine3d &pose,
        bool set_initial=false);

    /**
     * @brief Sets the values of a skeleton's joints
     *
     * @param robot_name The name of the skeleton
     *
     * @param joints The joint positions, velocities and efforts to be set
     *
     * @param set_initial Set the initial state of the robot to be here
     *                    so that when the simulation is reset it returns to
     *                    this pose
     *
     * @returns true if it succeeded else false
     */
    bool setJointStates(
        std::string robot_name, const sensor_msgs::JointState& joints,
        bool set_initial=false);

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
     * @brief Set up ROS services
     */
    void setupServices();

    /**
     * @brief Loads general simulation parameters
     */
    void loadGeneralParams();

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

    /**
     * @brief ROS service to pause the physics simulation
     */
    bool pauseService(
        std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp);

    /**
     * @brief ROS service to unpause the physics simulation
     */
    bool unpauseService(
        std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp);

    /**
     * @brief ROS service to step the physics simulation
     */
    bool stepService(
        std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp);

    /**
     * @brief ROS service to reset the world
     */
    bool resetService(
        std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp);

    /**
     * @brief ROS service to spawn a robot into the world
     */
    bool spawnRobotService(
        frea_dart_msgs::SpawnRobot::Request &req,
        frea_dart_msgs::SpawnRobot::Response &resp);

    /**
     * @brief ROS service to set the physical state of a robot
     */
    bool setRobotStateService(
        frea_dart_msgs::SetRobotState::Request &req,
        frea_dart_msgs::SetRobotState::Response &resp);

    /**
     * @brief ROS service to get the physical state of a robot
     */
    bool getRobotStateService(
        frea_dart_msgs::GetRobotState::Request &req,
        frea_dart_msgs::GetRobotState::Response &resp);

    ros::NodeHandle nh_;

    ros::ServiceServer pause_service_;
    ros::ServiceServer unpause_service_;
    ros::ServiceServer step_service_;
    ros::ServiceServer reset_service_;
    ros::ServiceServer spawn_robot_service_;
    ros::ServiceServer set_robot_state_service_;
    ros::ServiceServer get_robot_state_service_;

    dart::simulation::WorldPtr world_;
    // Dart won't let me use unique_ptr :(
    dart::gui::osg::WorldNode *world_node_;
    std::unique_ptr<dart::gui::osg::Viewer> viewer_;

    std::unique_ptr<RobotController> robot_ctrl_;

    ros::Publisher clock_pub_;

    unsigned long long timestep_;
    std::chrono::nanoseconds target_step_dur_;
    std::chrono::milliseconds render_period_;
    std::chrono::time_point<std::chrono::high_resolution_clock>
        last_render_update_;
    bool paused_;
    bool render_;
    ros::Time time_;

    std::unique_ptr<ros::AsyncSpinner> spinner_;
};

#endif // __FREA_SIMULATION_HPP__
