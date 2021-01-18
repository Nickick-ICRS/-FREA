#ifndef __LOAD_PARAM_HPP__
#define __LOAD_PARAM_HPP__

#include <string>
#include <ros/ros.h>
#include <sdf/sdf.hh>

/**
 * @brief Helper function to load parameters from SDF
 *
 * @param param_name The name of the parameter
 *
 * @param param Reference in which the output of the parameter is stored
 *
 * @param sdf Reference to the sdf to load the parameter from
 */
template<typename T>
void loadParam(std::string param_name, T &param, sdf::ElementPtr &sdf)
{
    if(sdf->HasElement(param_name)) {
        param = sdf->Get<T>(param_name);
    }
    else {
        ROS_WARN_STREAM(
            "AdjustableWeight failed to get SDF parameter '" << param_name
            << "'. Defaulting to " << param);
    }
}

#endif // __LOAD_PARAM_HPP__
