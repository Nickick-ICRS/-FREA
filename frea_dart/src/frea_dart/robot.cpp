#include "frea_dart/robot.hpp"

std::ostream& operator<<(std::ostream &os, const ControlType &type) {
    switch(type) {
    case ControlType::POSITION:
        os << "position_controllers";
        break;
    case ControlType::VELOCITY:
        os << "velocity_controllers";
        break;
    case ControlType::EFFORT:
        os << "effort_controllers";
        break;
    }

    return os;
}

Robot::Robot(const dart::dynamics::SkeletonPtr &skele) :skele_(skele) {
    ROS_ASSERT_MSG(skele != nullptr, "Cannot control a null-skeleton!");

    const auto &jnts = skele->getJoints();

    size_t num_jnts = 0;

    for(size_t i = 0; i < jnts.size(); i++) {
        const auto &jnt = jnts[i];
        if(jnt->getNumDofs() == 0) {
            continue;
        }
        if(jnt->getNumDofs() != 1) {
            ROS_INFO_STREAM(
                "Ignoring joint " << skele->getName() << "::"
                << jnt->getName()
                << " as only single DOF joints are supported (it has "
                << jnt->getNumDofs() << ")");
            continue;
        }
        // Store the joint's index
        jnt_idx_arr_.push_back(i);

        // Update how many joints we are controlling
        num_jnts++;
    }

    // Reserve memory for the joint state and command values
    cmd_.reset(new double[num_jnts]);
    pos_.reset(new double[num_jnts]);
    vel_.reset(new double[num_jnts]);
    eff_.reset(new double[num_jnts]);
    jnt_ctrl_type_.reset(new ControlType[num_jnts]);

    std::string param_ns = "/" + skele->getName() + "/controllers/";

    auto set_ctrl_type = [this](
        dart::dynamics::Joint *jnt, std::string type,
        ControlType default_type, size_t idx)
    {
        // Find out which control type we're using
        if(type.substr(0, 20) == "position_controllers") {
            jnt_ctrl_type_[idx] = ControlType::POSITION;
        }
        else if(type.substr(0, 20) == "velocity_controllers") {
            jnt_ctrl_type_[idx] = ControlType::VELOCITY;
        }
        else if(type.substr(0, 18) == "effort_controllers") {
            jnt_ctrl_type_[idx] = ControlType::EFFORT;
        }
        else {
            ROS_WARN_STREAM(
                "Unknown controller type: " << type << " defaulting to "
                << default_type);
            jnt_ctrl_type_[idx] = default_type;
        }

        // Now load the joint control interface
        hardware_interface::JointHandle jh;
        switch(jnt_ctrl_type_[idx]) {
        case ControlType::POSITION:
            jh = hardware_interface::JointHandle(
                jnt_state_interface_.getHandle(jnt->getName()), &cmd_[idx]);
            jnt_pos_interface_.registerHandle(jh);
            break;
        case ControlType::VELOCITY:
            jh = hardware_interface::JointHandle(
                jnt_state_interface_.getHandle(jnt->getName()), &cmd_[idx]);
            jnt_vel_interface_.registerHandle(jh);
            break;
        case ControlType::EFFORT:
            jh = hardware_interface::JointHandle(
                jnt_state_interface_.getHandle(jnt->getName()), &cmd_[idx]);
            jnt_eff_interface_.registerHandle(jh);
            break;
        }
    };

    for(size_t i = 0; i < num_jnts; i++) {
        cmd_[i] = 0;
        pos_[i] = 0;
        vel_[i] = 0;
        eff_[i] = 0;
        auto *jnt = skele->getJoint(jnt_idx_arr_[i]);

        // Register a handle for the joint
        hardware_interface::JointStateHandle state_handle(
            jnt->getName(), &pos_[i], &vel_[i], &eff_[i]);
        jnt_state_interface_.registerHandle(state_handle);

        std::string type;
        // First check and see if it's a position controller
        std::string param =
            param_ns + "position/" + jnt->getName() + "_controller/";
        if(ros::param::get(param + "type", type)) {
            // Create a position interface handle
            hardware_interface::JointHandle jh(
                jnt_state_interface_.getHandle(jnt->getName()), &cmd_[i]);
            jnt_pos_interface_.registerHandle(jh);
            set_ctrl_type(jnt, type, ControlType::POSITION, i);
            continue;
        }

        // Ok, try velocity control
        param = param_ns + "velocity/" + jnt->getName() + "_controller/";
        if(ros::param::get(param + "type", type)) {
            // Create a velocity interface handle
            hardware_interface::JointHandle jh(
                jnt_state_interface_.getHandle(jnt->getName()), &cmd_[i]);
            jnt_vel_interface_.registerHandle(jh);
            set_ctrl_type(jnt, type, ControlType::VELOCITY, i);
            continue;
        }

        // Ok, try effort control
        param = param_ns + "effort/" + jnt->getName() + "_controller/";
        if(ros::param::get(param + "type", type)) {
            // Create an effort interface handle
            set_ctrl_type(jnt, type, ControlType::EFFORT, i);
            continue;
        }

        // We don't know what type of controller it is!
        ROS_ERROR_STREAM(
            "Failed to load a controller for " << jnt->getName()
            << ". Expected something like '" << param << "'.");
        ignore_jnts_.push_back(i);
    }

    registerInterface(&jnt_state_interface_);
    registerInterface(&jnt_pos_interface_);
    registerInterface(&jnt_vel_interface_);
    registerInterface(&jnt_eff_interface_);
}

Robot::~Robot() {
    // dtor
}

void Robot::read() {
    for(size_t i = 0; i < jnt_idx_arr_.size(); i++) {
        bool skip = false;
        for(size_t j : ignore_jnts_) {
            if(j == i) {
                skip = true;
                break;
            }
        }
        if(skip)
            continue;

        auto *jnt = skele_->getJoint(jnt_idx_arr_[i]);

        pos_[i] = jnt->getPosition(0);
        vel_[i] = jnt->getVelocity(0);
        eff_[i] = jnt->getForce(0);
    }
}

void Robot::write() {
    for(size_t i = 0; i < jnt_idx_arr_.size(); i++) {
        bool skip = false;
        for(size_t j : ignore_jnts_) {
            if(j == i) {
                skip = true;
                break;
            }
        }
        if(skip)
            continue;

        auto *jnt = skele_->getJoint(jnt_idx_arr_[i]);

        switch(jnt_ctrl_type_[i]) {
        case ControlType::POSITION:
            jnt->setPosition(0, cmd_[i]);
            break;
        case ControlType::VELOCITY:
            jnt->setVelocity(0, cmd_[i]);
            break;
        case ControlType::EFFORT:
            jnt->setForce(0, cmd_[i]);
            break;
        }
    }
}
