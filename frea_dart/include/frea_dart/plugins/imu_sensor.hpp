#ifndef __IMU_SENSOR_HPP__
#define __IMU_SENSOR_HPP__

#include <random>

#include "frea_dart/plugins/dart_plugin.hpp"

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#include <dart/dart.hpp>

class ImuSensor :public DartPlugin {
public:
    /**
     * @brief Constructor
     *
     * @param robot The robot the sensor is applied to
     *
     * @param body_name The link the IMU is attached to
     *
     * @param topic_name The topic name of the IMU data
     *
     * @param update_rate The frequency of IMU measurement updates
     *
     * @param gaussian_noise Gaussian noise applied to the sensor readings
     */
    ImuSensor(
        const dart::dynamics::SkeletonPtr robot, std::string body_name,
        std::string topic_name, double update_rate, double gaussian_noise);

    /**
     * @brief Destructor
     */
    virtual ~ImuSensor();

    /**
     * @brief Update function, called every timestep
     *
     * @param dt The time (in s) since the previous update
     *
     * @param reset Signal to reset the sensor
     */
    virtual void update(double dt, bool reset) override;
protected:

    /**
     * @brief Calculates IMU measurements with noise
     *
     * @returns The IMU measurement
     */
    sensor_msgs::Imu calculateImuData();
    
    std::mt19937 gen_;
    std::normal_distribution<double> noise_;

    dart::dynamics::BodyNodePtr body_;

    ros::NodeHandle nh_;
    ros::Publisher imu_pub_;

    double update_period_;
    double time_since_last_update_;
    double variance_;
};

#endif // __IMU_SENSOR_HPP__
