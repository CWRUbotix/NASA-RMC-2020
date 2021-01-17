#ifndef OPEN_LOOP_MOVE_H_
#define OPEN_LOOP_MOVE_H_

#include <ros/ros.h>
#include <open_loop_move/OpenLoopMove.h> // Service definition
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Odometry.h>

class OpenLoopMove
{
public:
    OpenLoopMove(ros::NodeHandle &nh, ros::NodeHandle &pnh);

private:
    ros::NodeHandle nh_; // Nodehandle
    ros::NodeHandle pnh_; // Private nodehandle

    ros::ServiceServer move_srv_; // The service server
    ros::Publisher cmd_vel_pub_; // Publishes cmd vel
    ros::Publisher profile_pub_; // Publishes profile for debugging
    ros::Subscriber odom_sub_;  // Odometry to check move

    geometry_msgs::Point odom_pose_; // Odometry pose
    geometry_msgs::Twist odom_twist_; // Odometry twist
    double odom_yaw_; // Odometry yaw

    // Params
    bool publish_profile_; // Whether or not to publish profile

    // Trapazoidal profile params
    double max_vel_linear_;
    double max_accel_linear_;
    double max_vel_angular_;
    double max_accel_angular_;
    double dt_;

    // Position P gains during profile
    double kP_linear_;
    double kP_angular_;

    // Vel P gains during profile
    double kV_linear_;
    double kV_angular_;

    // Accel feedforward gains
    double kA_linear_;
    double kA_angular_;

    // Move service callback
    bool move_srv_cb(open_loop_move::OpenLoopMoveRequest &request, open_loop_move::OpenLoopMoveResponse &response);

    // Odom callback
    void odom_cb(const nav_msgs::OdometryConstPtr &msg);

    // Controls robot in a straight line
    void straight_path_controller(double start_x, double start_y, double target_pos, double target_vel, double target_accel);

    // Controls robot to turn
    void turn_in_place_controller(double start_angle, double target_pos, double target_vel, double target_accel);

    // Helper function to convert enum to string
    std::string enum_to_move_type(int move_type);

    // Helper function to convert planar quaternion to yaw
    double quat_to_yaw(geometry_msgs::Quaternion quat);
    
    // Helper funtion to convert angles to -pi to pi
    double min_angle(double angle);
};

#endif // OPEN_LOOP_MOVE_H_
