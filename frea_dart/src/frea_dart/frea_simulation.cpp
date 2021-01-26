#include "frea_dart/frea_simulation.hpp"

#include "frea_dart/utilities/load_param.hpp"

#include <dart/utils/urdf/urdf.hpp>
#include <dart/utils/utils.hpp>

#include <rosgraph_msgs/Clock.h>

#include <chrono>
#include <thread>

FreaSimulation::FreaSimulation() {
    loadWorld();
    setupWindow();
    setupPubSubs();
    reset();
}

FreaSimulation::~FreaSimulation() {
    // dtor
}

void FreaSimulation::setupWindow() {
    double width = 640;
    double height = 480;
    render_ = true;
    int timesteps_per_frame = 20;
    
    loadParam("/frea_dart/display/width", width);
    loadParam("/frea_dart/display/height", height);
    loadParam("/frea_dart/display/enable_rendering", render_);
    loadParam("/frea_dart/display/update_period", timesteps_per_frame);
    timesteps_per_frame_ = timesteps_per_frame;

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
}

void FreaSimulation::loadSkeletonParam(
    std::string robot_description, bool ctrl)
{
    std::string desc;
    loadParam(robot_description, desc);

    if(desc == "") return;

    dart::utils::DartLoader loader;
    dart::dynamics::SkeletonPtr skele = loader.parseSkeletonString(
        desc, robot_description);

    if(!skele) {
        ROS_ERROR_STREAM(
            "Failed to load skeleton from " << robot_description);
        return;
    }

    ROS_INFO_STREAM("Loaded skeleton '" << skele->getName() << "'");
    world_->addSkeleton(skele);

    if(ctrl)
        robot_ctrl_.reset(new RobotController(skele));
}

void FreaSimulation::loadSkeletonFile(std::string filepath, bool ctrl) {
    dart::utils::DartLoader loader;
    dart::dynamics::SkeletonPtr skele = loader.parseSkeleton(filepath);

    if(!skele) {
        ROS_ERROR_STREAM(
            "Failed to load skeleton from " << filepath);
        return;
    }

    ROS_INFO_STREAM("Loaded skeleton '" << skele->getName() << "'");
    world_->addSkeleton(skele);

    if(ctrl)
        robot_ctrl_.reset(new RobotController(skele));
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

void FreaSimulation::reset() {
    timestep_ = 0;
}

void FreaSimulation::step() {
    auto step_start = std::chrono::high_resolution_clock::now();
    //ROS_WARN_STREAM("T: " << timestep_);

    world_->step();

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
    auto dt =
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::high_resolution_clock::now() - now);
    //ROS_INFO_STREAM("Controller update took " << dt.count() / 1e9 << "s");

    if(render_) {
        render();
    }

    timestep_++;

    dt =
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::high_resolution_clock::now() - step_start);
    //ROS_INFO_STREAM("Step took " << dt.count() / 1e9 << "s");
}

void FreaSimulation::render() {
    if(timestep_ % timesteps_per_frame_ == 0) {
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
        step();

        if(windowClosed())
            return;
    }
}

bool FreaSimulation::setSkeletonPose(
    const std::string name, const Eigen::Affine3d &pose)
{
    dart::dynamics::SkeletonPtr skele = world_->getSkeleton(name);

    if(!skele) {
        ROS_ERROR_STREAM("Skeleton '" << name << "' does not exist!");
        return false;
    }

    // Assume the first joint is the connection to the world
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

    ROS_INFO_STREAM(
        "Set Joint " << joint->getName() << " to ["
        << pose.translation().x() << ", " << pose.translation().y() << ", "
        << pose.translation().z() << "], [" << ea[0] << ", " << ea[1]
        << ", " << ea[2] << "]");

    return true;
}
