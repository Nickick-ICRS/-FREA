#ifndef __LOAD_PARAM_HPP__
#define __LOAD_PARAM_HPP__

#include <string>

#include <ros/ros.h>

template<class T>
void loadParam(const std::string &param_name, T &val) {
    if(!ros::param::get(param_name, val)) {
        ROS_WARN_STREAM(
            "Failed to load '" << param_name << "'. Defaulting to " << val);
    }
}

#endif // __LOAD_PARAM_HPP__
