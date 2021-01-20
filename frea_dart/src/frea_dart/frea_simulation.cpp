#include "frea_dart/frea_simulation.hpp"

#include <dart/utils/urdf/urdf.hpp>
#include <dart/utils/utils.hpp>

#include <ros/callback_queue.h>

template<class T>
void loadParam(const std::string &param_name, T &val) {
    if(!ros::param::get(param_name, val)) {
        ROS_WARN_STREAM(
            "Failed to load '" << param_name << "'. Defaulting to " << val);
    }
}

FreaSimulation::FreaSimulation() {
    load_world();
    setup_window();
    reset();
}

FreaSimulation::~FreaSimulation() {
// Dart is naughty and stores the raw pointer I gave it in a shared pointer
//    if (world_node_)
//        delete world_node_;
}

void FreaSimulation::setup_window() {
    double width = 640;
    double height = 480;
    render_ = true;
    int timesteps_per_frame = 20;
    
    loadParam("/frea_dart/display/width", width);
    loadParam("/frea_dart/display/height", height);
    loadParam("/frea_dart/display/enable_rendering", render_);
    loadParam("/frea_dart/display/update_frequency", timesteps_per_frame);
    timesteps_per_frame_ = timesteps_per_frame;

    world_node_ = new dart::gui::osg::WorldNode(world_);
    viewer_.reset(new dart::gui::osg::Viewer());
    viewer_->addWorldNode(world_node_);
    viewer_->setUpViewInWindow(0, 0, width, height);
    osg::Vec3d eye(1000, 0, 1000);
    osg::Vec3d center(0, 0, 0);
    osg::Vec3d up(0.707, 0, 0.707);
    viewer_->getCamera()->setViewMatrixAsLookAt(eye, center, up);
}

void FreaSimulation::load_skeleton_param(std::string robot_description) {
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

    world_->addSkeleton(skele);
}

void FreaSimulation::load_skeleton_file(std::string filepath) {
    dart::utils::DartLoader loader;
    dart::dynamics::SkeletonPtr skele = loader.parseSkeleton(filepath);

    if(!skele) {
        ROS_ERROR_STREAM(
            "Failed to load skeleton from " << filepath);
        return;
    }

    world_->addSkeleton(skele);
}

void FreaSimulation::load_world() {
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

void FreaSimulation::reset() {
    timestep_ = 0;
}

void FreaSimulation::step() {
    timestep_++;

    if(render_) {
        render();
    }
}

void FreaSimulation::render() {
    if(timestep_ % timesteps_per_frame_ == 0) {
        viewer_->frame();
    }
}

void FreaSimulation::run() {
    auto q = ros::getGlobalCallbackQueue();
    while(ros::ok()) {
        step();
        // Handle ROS callbacks etc. Do not wait for new ones as we publish
        // the clock...
        q->callAvailable();
    }
}
