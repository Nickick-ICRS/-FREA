#ifndef __BASE_LINK_TF_CALCULATOR_HPP__
#define __BASE_LINK_TF_CALCULATOR_HPP__

#include <deque>
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <sensor_msgs/Imu.h>

namespace frea_motion_control {

/**
 * @brief Class to calculate the transform from the base_link to the
 *        chassis_link
 *
 * @details Relies upon accurate IMU data (default topic /imu/data)
 *          to describe the tilt of the base of the robot, thus keeping
 *          the virtual "base_link" frame parallel to the ground.
 *
 *          It is assumed that the IMU frame is aligned with the
 *          chassis_link frame.
 */
class BaseLinkTfCalculator {
public:
    /**
     * @brief Constructor
     */
    BaseLinkTfCalculator();

    /**
     * @brief Destructor
     */
    ~BaseLinkTfCalculator();

    /**
     * @brief Spin function. Sets up topic connections and calls ros::spin()
     */
    void spin();
private:
    /**
     * @brief Callback for IMU message
     *
     * @details Calculates and publishes the transform from base_link to
     *          chassis_link based on the IMU message data.
     *
     * @param msg The IMU message
     */
    void imuCallback(const sensor_msgs::ImuConstPtr &msg);

    /**
     * @brief Calculates the moving average of the pitch
     *
     * @details Updates the stored pitch values before calculating the
     *          moving average. Up to ma_history_length values will be
     *          stored.
     *
     * @param pitch The latest pitch reading from the IMU.
     *
     * @returns The average pitch.
     */
    double calculatePitchMA(double pitch);

    double updateKFPitchEstimate(double pitch, double variance);

    /**
     * @brief ROS NodeHandle
     */
    ros::NodeHandle nh_;

    /**
     * @brief Subscriber to IMU data
     */
    ros::Subscriber imu_sub_;

    /**
     * @brief TF2 Broadcaster for frame transform publication
     */
    tf2_ros::TransformBroadcaster tf2_br_;

    /**
     * @brief Double ended queue to store the last X readings of pitch
     *        from the IMU.
     */
    std::deque<double> pitch_values_;

    /**
     * @brief Name of the base_link in TF
     */
    std::string base_link_;

    /**
     * @brief Name of the chassis_link in TF
     */
    std::string chassis_link_;

    /**
     * @brief Maximum number of previous readings to consider when
     *        calculating the moving average of the IMU reading.
     */
    int ma_history_length_;
};

}; // ns frea_motion_control

#endif // __BASE_LINK_TF_CALCULATOR_HPP__
