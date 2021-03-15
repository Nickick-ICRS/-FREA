#include "frea_dart/frea_simulation.hpp"

#include "frea_dart/utilities/load_param.hpp"

#include <dart/utils/urdf/urdf.hpp>
#include <dart/utils/utils.hpp>

#include <rosgraph_msgs/Clock.h>
#include <eigen_conversions/eigen_msg.h>
#include <urdf/model.h>

#include <chrono>
#include <thread>

FreaSimulation::FreaSimulation() {
    loadGeneralParams();
    loadWorld();
    setupWindow();
    setupPubSubs();
    setupServices();
    reset();
}

FreaSimulation::~FreaSimulation() {
    // dtor
}

void FreaSimulation::loadGeneralParams() {
    double steps_per_s = 1000;
    loadParam("/frea_dart/simulation/steps_per_s", steps_per_s);
    if(steps_per_s > 0) {
    target_step_dur_ =
        std::chrono::nanoseconds((long long)(1e9 / steps_per_s));
    }
    else {
        target_step_dur_ = std::chrono::nanoseconds(0);
    }

    paused_ = false;
    loadParam("/frea_dart/simulation/start_paused", paused_);
}

void FreaSimulation::setupWindow() {
    double width = 640;
    double height = 480;
    render_ = true;
    double fps = 30;
    last_render_update_ = std::chrono::high_resolution_clock::now();
    
    loadParam("/frea_dart/display/width", width);
    loadParam("/frea_dart/display/height", height);
    loadParam("/frea_dart/display/enable_rendering", render_);
    loadParam("/frea_dart/display/fps", fps);
    render_period_ = std::chrono::milliseconds((long)(1e3 / fps));

    world_node_ = new dart::gui::osg::WorldNode(world_);
    viewer_.reset(new dart::gui::osg::Viewer());
    viewer_->addWorldNode(world_node_);
    viewer_->setUpViewInWindow(0, 0, width, height);

    viewer_->realize();

    osg::Vec3d eye(-3, 0, 1.8); // Eye position
    osg::Vec3d center(0, 0, 0); // What we're looking at
    osg::Vec3d up(0, 0, 1); // Up vector
    viewer_->getCameraManipulator()->setHomePosition(eye, center, up);
    // Tell the viewer to update the camera home position
    viewer_->setCameraManipulator(viewer_->getCameraManipulator());
}

void FreaSimulation::setupPubSubs() {
    clock_pub_ = nh_.advertise<rosgraph_msgs::Clock>("/clock", 1);

    // Set up ROS spinner with 4 threads
    spinner_.reset(new ros::AsyncSpinner(4));
    spinner_->start();
}

void FreaSimulation::setupServices() {
    pause_service_ = nh_.advertiseService(
        "/frea_dart/pause", &FreaSimulation::pauseService, this);
    unpause_service_ = nh_.advertiseService(
        "/frea_dart/unpause", &FreaSimulation::unpauseService, this);
    step_service_ = nh_.advertiseService(
        "/frea_dart/step", &FreaSimulation::stepService, this);
    reset_service_ = nh_.advertiseService(
        "/frea_dart/reset", &FreaSimulation::resetService, this);
    spawn_robot_service_ = nh_.advertiseService(
        "/frea_dart/spawn_robot", &FreaSimulation::spawnRobotService, this);
    get_body_state_service_ = nh_.advertiseService(
        "/frea_dart/get_body_state",
        &FreaSimulation::getBodyStateService,
        this);
    set_robot_state_service_ = nh_.advertiseService(
        "/frea_dart/set_robot_state",
        &FreaSimulation::setRobotStateService,
        this);
    get_robot_state_service_ = nh_.advertiseService(
        "/frea_dart/get_robot_state",
        &FreaSimulation::getRobotStateService,
        this);
}

bool FreaSimulation::loadSkeletonParam(
    std::string robot_description, bool ctrl)
{
    std::string desc;
    loadParam(robot_description, desc);

    if(desc == "") return false;

    dart::utils::DartLoader loader;
    dart::dynamics::SkeletonPtr skele = loader.parseSkeletonString(
        desc, robot_description);

    if(!skele) {
        ROS_ERROR_STREAM(
            "Failed to load skeleton from " << robot_description);
        return false;
    }

    skele->setSelfCollisionCheck(true);

    ROS_INFO_STREAM("Loaded skeleton '" << skele->getName() << "'");
    world_->addSkeleton(skele);

    if(ctrl) {
        auto urdf = std::make_shared<urdf::Model>();
        urdf->initParam(robot_description);
        robot_ctrl_.reset(new RobotController(skele, urdf));
    }

    plugin_manager_.loadPlugins(world_, skele);
    return true;
}

bool FreaSimulation::loadSkeletonFile(std::string filepath, bool ctrl) {
    dart::utils::DartLoader loader;
    dart::dynamics::SkeletonPtr skele = loader.parseSkeleton(filepath);

    if(!skele) {
        ROS_ERROR_STREAM(
            "Failed to load skeleton from " << filepath);
        return false;
    }

    skele->setSelfCollisionCheck(false);

    ROS_INFO_STREAM("Loaded skeleton '" << skele->getName() << "'");
    world_->addSkeleton(skele);

    if(ctrl) {
        ROS_WARN_STREAM(
            "ROS Control only supported for robots loaded via parameter"
            << " server");
        // robot_ctrl_.reset(new RobotController(skele));
    }

    return true;
}

void FreaSimulation::loadWorld() {
    std::string world_filepath;
    loadParam("/frea_dart/world/filepath", world_filepath);

    if(world_filepath.length() > 5) {
        std::string filetype = 
            world_filepath.substr(world_filepath.length()-5);
        if(filetype == ".skel") {
            ROS_INFO_STREAM("Loading .skel world: " << world_filepath);
            world_ = dart::utils::SkelParser::readWorld(world_filepath);
        }
        else if(filetype == ".urdf") {
            dart::utils::DartLoader loader;
            ROS_INFO_STREAM("Loading .urdf world: " << world_filepath);
            world_ = loader.parseWorld(world_filepath);
        }
        else {
            ROS_WARN_STREAM("Unknown world type: " << world_filepath);
            world_.reset(new dart::simulation::World());
        }
        if(!world_) {
            ROS_ERROR_STREAM("Failed to load world: " << world_filepath);
            world_.reset(new dart::simulation::World());
        }
    }
    else {
        ROS_WARN("No world file given.");
        world_.reset(new dart::simulation::World());
    }
    if(world_->getGravity() == Eigen::Vector3d(0, 0, 0)) {
        world_->setGravity(Eigen::Vector3d(0, 0, -9.8));
    }

    if(world_->getTimeStep() == 0) {
        world_->setTimeStep(0.001);
        ROS_WARN_STREAM(
            "No timestep specified in world file, defaulting to "
            << world_->getTimeStep());
    }
}

bool FreaSimulation::windowClosed() {
    if(!render_)
        return false;

    return !viewer_->isRealized();
}

void FreaSimulation::reset(bool reset_time, bool reset_robots) {
    if(reset_time)
        timestep_ = 0;

    bool prev_paused = paused_;
    paused_ = true;

    std::this_thread::sleep_for(target_step_dur_);

    world_->reset();

    // Reset the positions, velocities and efforts of all joints of all
    // robots
    if(reset_robots) {
        size_t num_skeletons = world_->getNumSkeletons();
        for(size_t i = 0; i < num_skeletons; i++) {
            const auto &skele = world_->getSkeleton(i);
            skele->resetPositions();
            skele->resetVelocities();
            skele->resetAccelerations();
            skele->resetCommands();
            skele->resetGeneralizedForces();
        }

        step();
        if(robot_ctrl_) {
            robot_ctrl_->update(
                time_, ros::Duration(world_->getTimeStep()), true);
        }
        plugin_manager_.update(0, true);
    }

    std::this_thread::sleep_for(target_step_dur_);

    paused_ = prev_paused;
}

void FreaSimulation::step() {
    auto step_start = std::chrono::high_resolution_clock::now();
    //ROS_WARN_STREAM("T: " << timestep_);

    world_->step(false);

    publishTime();

    auto now = std::chrono::high_resolution_clock::now();
    if(robot_ctrl_) {
        // Reset if timestep is 0
        if(timestep_ == 0)
            robot_ctrl_->update(
                time_, ros::Duration(world_->getTimeStep()), true);
        else
            robot_ctrl_->update(
                time_, ros::Duration(world_->getTimeStep()), false);
    }
    if(timestep_ == 0)
        plugin_manager_.update(world_->getTimeStep(), true);
    else
        plugin_manager_.update(world_->getTimeStep(), false);

    timestep_++;
}

void FreaSimulation::render() {
    auto now = std::chrono::high_resolution_clock::now();
    if(now - last_render_update_ >= render_period_) {
        last_render_update_ = now;

        // Little hack to stop it from randomly crashing
        if(viewer_->getCamera()->getViewMatrix().isNaN()) {
            ROS_WARN("Resetting camera view matrix due to NaNs");
            viewer_->getCamera()->setViewMatrix(osg::Matrix::identity());
        }
        viewer_->frame();
    }
}

void FreaSimulation::publishTime() {
    time_ += ros::Duration(world_->getTimeStep());

    if(timestep_ == 0)
        time_ = ros::Time(0);

    rosgraph_msgs::Clock clock;
    clock.clock = time_;
    clock_pub_.publish(clock);
}

void FreaSimulation::run() {
    while(ros::ok()) {
        auto start = std::chrono::high_resolution_clock::now();
        if(!paused_) {
            step();
        }

        if(render_) {
            render();
        }

        if(windowClosed())
            return;

        // Wait until the time has elapsed before the next frame update
        while(std::chrono::high_resolution_clock::now() - start < target_step_dur_);
    }
}

bool FreaSimulation::setSkeletonPose(
    const std::string name, const Eigen::Affine3d &pose, bool set_initial)
{
    dart::dynamics::SkeletonPtr skele = world_->getSkeleton(name);

    if(!skele) {
        ROS_ERROR_STREAM("Skeleton '" << name << "' does not exist!");
        return false;
    }

    dart::dynamics::JointPtr joint = skele->getJoint(0);

    if(!joint) {
        ROS_ERROR_STREAM(name << " has no joints!");
        return false;
    }
    if(joint->getType() != "FreeJoint") {
        ROS_ERROR_STREAM(
            "Could not find FreeJoint between " << name
            << " and the world. Joint " << joint->getName() << "'s type is "
            << joint->getType());
        return false;
    }


    joint->setPosition(3, pose.translation().x());
    joint->setPosition(4, pose.translation().y());
    joint->setPosition(5, pose.translation().z());

    auto ea = pose.rotation().eulerAngles(0, 1, 2);

    joint->setPosition(0, ea[0]);
    joint->setPosition(1, ea[1]);
    joint->setPosition(2, ea[2]);

    if(set_initial) {
        auto initial_positions = joint->getPositions();
        joint->setInitialPositions(initial_positions);
    }

    return true;
}

bool FreaSimulation::setJointStates(
    std::string robot_name, const sensor_msgs::JointState& joints,
    bool set_initial)
{
    dart::dynamics::SkeletonPtr skele = world_->getSkeleton(robot_name);

    if(!skele) {
        ROS_ERROR_STREAM("Skeleton '" << robot_name << "' does not exist!");
        return false;
    }
    
    for(size_t i = 0; i < joints.name.size(); i++) {
        dart::dynamics::Joint *jnt = skele->getJoint(joints.name[i]);
        if(!jnt) {
            ROS_WARN_STREAM(
                skele->getName() << " does not have a joint called "
                << joints.name[i]);
            continue;
        }

        if(jnt->getNumDofs() != 1) {
            ROS_WARN_STREAM(
                "Ignoring joint " << jnt->getName() << " as it has "
                << jnt->getNumDofs() << " (only 1 is supported).");
        }

        jnt->setPosition(0, joints.position[i]);
        jnt->setVelocity(0, joints.velocity[i]);
        jnt->setForce(0, joints.effort[i]);

        if(set_initial) {
            jnt->setInitialPosition(0, joints.position[i]);
            jnt->setInitialVelocity(0, joints.velocity[i]);
        }
    }

    return true;
}

bool FreaSimulation::pauseService(
    std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp)
{
    paused_ = true;
    return true;
}

bool FreaSimulation::unpauseService(
    std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp)
{
    paused_ = false;
    return true;
}

bool FreaSimulation::stepService(
    std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp)
{
    step();
    return true;
}

bool FreaSimulation::resetService(
    std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp)
{
    reset(false, true);
    return true;
}

bool FreaSimulation::spawnRobotService(
    frea_dart_msgs::SpawnRobot::Request &req,
    frea_dart_msgs::SpawnRobot::Response &resp)
{
    if(!loadSkeletonParam(req.urdf_parameter)) {
        resp.success = false;
        resp.status_message =
            "Failed to load skeleton from " + req.urdf_parameter;
        return false;
    }

    Eigen::Affine3d pose;
    tf::poseMsgToEigen(req.initial_pose, pose);
    if(!setSkeletonPose(req.model_name, pose, true)) {
        resp.success = false;
        resp.status_message =
            "Failed to set pose of " + req.model_name +
            ". Does it match the robot name in " + req.urdf_parameter + "?";
        return false;
    }

    resp.success = true;
    return true;
}

bool FreaSimulation::getBodyStateService(
    frea_dart_msgs::GetBodyState::Request &req,
    frea_dart_msgs::GetBodyState::Response &resp)
{
    dart::dynamics::SkeletonPtr skele = world_->getSkeleton(req.robot_name);
    if(!skele) {
        resp.success = false;
        ROS_ERROR_STREAM(
            "Skeleton '" << req.robot_name << "' does not exist!");
        resp.status_message =
            "Failed to get state of " + req.robot_name + " does it exist?";
        return false;
    }

    dart::dynamics::BodyNodePtr body = skele->getBodyNode(req.body_name);
    if(!body) {
        resp.success = false;
        ROS_ERROR_STREAM(
            "Body '" << req.body_name << "' does not exist in robot "
            << req.robot_name << "!");
        resp.status_message =
            "Failed to get state of " + req.robot_name + "::"
            + req.body_name + " does it exist?";
        return false;
    }

    dart::dynamics::Frame* frame;
    if(req.reference_frame == "world") {
        frame = dart::dynamics::Frame::World();
    }
    else {
        frame = world_->getSimpleFrame(req.reference_frame).get();
    }
    if(!frame) {
        resp.success = false;
        ROS_ERROR_STREAM(
            "Frame '" << req.reference_frame << "' does not exist!");
        resp.status_message =
            "Failed to get frame " + req.reference_frame
            + " does it exist?";
        return false;
    }

    Eigen::Affine3d pose = body->getTransform(frame);
    tf::poseEigenToMsg(pose, resp.pose);

    Eigen::Vector6d vels = body->getSpatialVelocity(frame, frame);
    resp.twist.angular.x = vels[0];
    resp.twist.angular.y = vels[1];
    resp.twist.angular.z = vels[2];
    resp.twist.linear.x = vels[3];
    resp.twist.linear.y = vels[4];
    resp.twist.linear.z = vels[5];

    resp.success = true;
    return true;
}

bool FreaSimulation::setRobotStateService(
    frea_dart_msgs::SetRobotState::Request &req,
    frea_dart_msgs::SetRobotState::Response &resp)
{
    Eigen::Affine3d pose;
    tf::poseMsgToEigen(req.pose, pose);
    if(!setSkeletonPose(req.robot_name, pose, req.update_initial_state)) {
        resp.success = false;
        resp.status_message =
            "Failed to set pose of " + req.robot_name + " does it exist?";
        return false;
    }

    if(!setJointStates(
        req.robot_name, req.joints, req.update_initial_state))
    {
        resp.success = false;
        resp.status_message = "Failed to set joints of " + req.robot_name;
        return false;
    }

    resp.success = true;
    return true;
}

bool FreaSimulation::getRobotStateService(
    frea_dart_msgs::GetRobotState::Request &req,
    frea_dart_msgs::GetRobotState::Response &resp)
{
    dart::dynamics::SkeletonPtr skele = world_->getSkeleton(req.robot_name);

    if(!skele) {
        resp.success = false;
        ROS_ERROR_STREAM(
            "Skeleton '" << req.robot_name << "' does not exist!");
        resp.status_message =
            "Failed to get state of " + req.robot_name + " does it exist?";
        return false;
    }

    double X, Y, Z, r, p, y;
    r = skele->getPosition(0);
    p = skele->getPosition(1);
    y = skele->getPosition(2);
    X = skele->getPosition(3);
    Y = skele->getPosition(4);
    Z = skele->getPosition(5);

    Eigen::Affine3d pose;
    pose = Eigen::Translation3d(X, Y, Z)
           * Eigen::AngleAxisd(y, Eigen::Vector3d::UnitZ())
           * Eigen::AngleAxisd(p, Eigen::Vector3d::UnitY())
           * Eigen::AngleAxisd(r, Eigen::Vector3d::UnitX());

    resp.header.frame_id = "world";
    resp.header.stamp = ros::Time::now();

    tf::poseEigenToMsg(pose, resp.pose);

    size_t num_jnts = skele->getNumJoints();
    for(size_t i = 0; i < num_jnts; i++) {
        auto *jnt = skele->getJoint(i);

        if(jnt->getNumDofs() != 1)
            continue;

        resp.joints.name.push_back(jnt->getName());
        resp.joints.position.push_back(jnt->getPosition(0));
        resp.joints.velocity.push_back(jnt->getVelocity(0));
        resp.joints.effort.push_back(jnt->getForce(0));
    }

    resp.success = true;
    return true;
}
