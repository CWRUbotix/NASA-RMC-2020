#ifndef IMU_CALIBRATOR_H_
#define IMU_CALIBRATOR_H_

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_srvs/Trigger.h>

class IMUCalibrator
{
public:
    // Constructor
    IMUCalibrator(ros::NodeHandle &nh, ros::NodeHandle &pnh); 

private:
    // Imu message callback
    void imu_cb(const sensor_msgs::ImuConstPtr &msg);

    // Start calibrating callback
    bool calibrate_service_cb(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &resp);

    // Check if robot is still by looking at imu
    bool is_still(std::vector<double> &samples);

    // Helper function to get average
    double mean(std::vector<double> &samples);
    
    ros::NodeHandle nh_;  // Nodehandle
    ros::NodeHandle pnh_;  // Private Nodehandle

    ros::Subscriber imu_sub_;  //Subscriber for imu
    ros::Publisher imu_pub_;  // Calibrated imu publisher
    ros::Publisher bias_pub_;  // Found bias publisher
    ros::ServiceServer cali_srv_;  // Calibrate service

    geometry_msgs::Vector3 bias_;  // rpy biases
    std::vector<double> roll_samples_;  // Average to find biases
    std::vector<double> pitch_samples_;  // Average to find biases
    std::vector<double> yaw_samples_;  // Average to find biases

    bool calibrating_;  // If calibration process enabled

    // Params
    double still_range_;  // Considered still if range less than threshold
    double calib_time_;  // How long to calibrate for

};

#endif // IMU_CALIBRATOR_H_