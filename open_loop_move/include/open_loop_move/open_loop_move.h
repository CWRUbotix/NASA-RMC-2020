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

    // Move service callback
    bool move_srv_cb(open_loop_move::OpenLoopMoveRequest &request, open_loop_move::OpenLoopMoveResponse &response);

    // Odom callback
    void odom_cb(const nav_msgs::OdometryConstPtr &msg);

    // Controls robot in a straight line
    void straight_path_controller(double pos, double vel, double accel);

    // Controls robot to turn
    void turn_in_place_controller(double pos, double vel, double accel);

    // Helper function to convert enum to string
    std::string enum_to_move_type(int move_type);

    // Helper function to convert planar quaternion to yaw
    double quat_to_yaw(geometry_msgs::Quaternion quat);
    
    // Helper funtion to convert angles to -pi to pi
    double min_angle(double angle);
};

#endif // OPEN_LOOP_MOVE_H_
