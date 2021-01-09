#include <imu_calibrator/imu_calibrator.h>

IMUCalibrator::IMUCalibrator(ros::NodeHandle &nh, ros::NodeHandle &pnh) : nh_(nh), pnh_(pnh) 
{
    // When this node is in a namespace, this will be
    // published as /imu/data, for example
    imu_sub_ = nh_.subscribe<sensor_msgs::Imu>("data_raw", 1, &IMUCalibrator::imu_cb, this);
    imu_pub_ = nh_.advertise<sensor_msgs::Imu>("data", 1);
    bias_pub_ = nh_.advertise<geometry_msgs::Vector3>("bias", 1, true);

    cali_srv_ = nh_.advertiseService("calibrate", &IMUCalibrator::calibrate_service_cb, this);

    // Read paramters
    pnh_.param("calibrate_time", calib_time_, 1.0);
    pnh_.param("still_range", still_range_, 0.1);

    pnh_.param("default_bias_x", bias_.x, 0.0);
    pnh_.param("default_bias_y", bias_.y, 0.0);
    pnh_.param("default_bias_z", bias_.z, 0.0);

    ROS_INFO("Calibrate time set to %.2f", calib_time_);
    ROS_INFO("Still range set to %.2f", still_range_);
    ROS_INFO("Initial bias set to %.3f, %.3f, %.3f", bias_.x, bias_.y, bias_.z);

    calibrating_ = false;

    ROS_INFO("IMU Calibrator node initialized");
}

void IMUCalibrator::imu_cb(const sensor_msgs::ImuConstPtr &msg)
{
    if (calibrating_)
    {
        // Save history of samples
        roll_samples_.push_back(msg->angular_velocity.x);
        pitch_samples_.push_back(msg->angular_velocity.y);
        yaw_samples_.push_back(msg->angular_velocity.z);
    }

    // Copy and republish message
    sensor_msgs::Imu out_msg;
    out_msg.header = msg->header;
    out_msg.orientation = msg->orientation;
    out_msg.orientation_covariance = msg->orientation_covariance;
    out_msg.angular_velocity = msg->angular_velocity;
    out_msg.angular_velocity_covariance = msg->angular_velocity_covariance;
    out_msg.linear_acceleration = msg->linear_acceleration;
    out_msg.linear_acceleration_covariance = msg->linear_acceleration_covariance;
    out_msg.angular_velocity.x -= bias_.x;    
    out_msg.angular_velocity.y -= bias_.y;
    out_msg.angular_velocity.z -= bias_.z;

    imu_pub_.publish(out_msg);
}

bool IMUCalibrator::is_still(std::vector<double> &samples)
{
    double max = -10;
    double min = 10;
    for(double item : samples)
    {
        if (item > max) {
            max = item;
        }
        if (item < min) {
            min = item;
        }
    }

    // Check for small range to indicate stillness
    return (max - min) < still_range_;
}

double IMUCalibrator::mean(std::vector<double> &samples)
{
    double sum = 0;
    for(double item : samples)
    {
       sum += item;
    }

    return sum / samples.size();
}


bool IMUCalibrator::calibrate_service_cb(std_srvs::TriggerRequest &request, std_srvs::TriggerResponse &response)
{
    ROS_INFO("Calibrating imu...");
    calibrating_ = true;

    bool success = false;
    int tries = 0;
    int num_tries = 3;

    // Frequency to poll
    ros::Rate timer(200.0);
    while(!success && tries < num_tries && ros::ok())
    {
        // Pause to collect samples
        // Need to spin so subscriber still works
        ros::Time start_time = ros::Time::now();
        while(ros::Time::now() - start_time < ros::Duration(calib_time_) && ros::ok()){
            timer.sleep();
            ros::spinOnce();
        }
        tries++;
        if (roll_samples_.size() == 0)
        {
            ROS_WARN("Did not recieve any imu messages, trying again");
        }
        else if (is_still(roll_samples_) && is_still(pitch_samples_) && is_still(yaw_samples_))
        {
            // If the robot was sitting still
            // we can find the bias of the imu
            success = true;
            bias_.x = mean(roll_samples_);
            bias_.y = mean(pitch_samples_);
            bias_.z = mean(yaw_samples_);
        }
        else {
            ROS_WARN("Too much movement, trying again");
        }
    }

    if (!success)
    {
        ROS_ERROR("IMU Calibration failed, exiting");
    }
    else {
        ROS_INFO("Successfully calibrated");
        ROS_INFO("New biases %.3f, %.3f, %.3f", bias_.x, bias_.y, bias_.z);
        bias_pub_.publish(bias_);  // Publish new bias for debugging
    }

    // Reset for next time
    roll_samples_.clear();
    pitch_samples_.clear();
    yaw_samples_.clear();
    calibrating_ = false;

    // Tell service if calibration succeeded
    response.success = success;

    return true;
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "imu_calibrator");
    ros::NodeHandle nh("");
    ros::NodeHandle pnh("~");

    IMUCalibrator calibrator(nh, pnh);

    ros::spin();

    return 0;
}