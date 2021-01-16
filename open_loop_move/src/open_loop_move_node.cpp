#include <open_loop_move/open_loop_move.h>
#include <open_loop_move/trapezoidal_profile.h>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

OpenLoopMove::OpenLoopMove(ros::NodeHandle &nh, ros::NodeHandle &pnh) : nh_(nh), pnh_(pnh)
{
    ROS_INFO("Open Loop Move node initialized");
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    move_srv_ = nh_.advertiseService("move_open_loop", &OpenLoopMove::move_srv_cb, this);
    odom_sub_ = nh_.subscribe("odom", 1, &OpenLoopMove::odom_cb, this);

    // Read params
    double update_rate;
    pnh_.param("dt", update_rate, 20.0);
    dt_ = 1.0 / update_rate;

    pnh_.param("max_vel_linear", max_vel_linear_, 0.5);
    pnh_.param("max_accel_linear", max_accel_linear_, 1.5);
    pnh_.param("max_vel_angular", max_vel_angular_, 1.5);
    pnh_.param("max_accel_angular", max_accel_angular_, 3.0);

    pnh_.param("kP_linear", kP_linear_, 0.0);
    pnh_.param("kP_angular", kP_angular_, 0.0);
    pnh_.param("kV_linear", kV_linear_, 0.0);
    pnh_.param("kV_angular", kV_angular_, 0.0);
    pnh_.param("kA_linear", kA_linear_, 0.0);
    pnh_.param("kA_angular", kA_angular_, 0.0);

    pnh_.param("publish_profile", publish_profile_, false);

    if (publish_profile_)
    {
        profile_pub_ = pnh_.advertise<geometry_msgs::Point>("profile", 1);
    }
}

std::string OpenLoopMove::enum_to_move_type(int move_type)
{
    if (move_type == open_loop_move::OpenLoopMoveRequest::DISTANCE)
    {
        return "Distance";
    }
    else if (move_type == open_loop_move::OpenLoopMoveRequest::ANGLE)
    {
        return "To angle";
    }

    return "Unknown";
}

double OpenLoopMove::quat_to_yaw(geometry_msgs::Quaternion quat)
{
    // This is the math for quats with just yaw
    return 2.0 * atan2(quat.z, quat.w);
}

// Convert any angle to be between -pi and pi
double OpenLoopMove::min_angle(double angle) {
    if (angle > M_PI) {
        angle -= 2.0 * M_PI;}
    if (angle< -M_PI) {
        angle += 2.0 * M_PI;}
    return angle;   
}

void OpenLoopMove::odom_cb(const nav_msgs::OdometryConstPtr &msg)
{
    odom_pose_ = msg->pose.pose.position;
    odom_twist_ = msg->twist.twist;
    odom_yaw_ = quat_to_yaw(msg->pose.pose.orientation);
}

bool OpenLoopMove::move_srv_cb(open_loop_move::OpenLoopMoveRequest &request, open_loop_move::OpenLoopMoveResponse &response)
{
    double distance = request.distance;
    int move_type = request.move_type;

    ROS_INFO_STREAM(enum_to_move_type(move_type) << " move requested of distance " << distance);

    std::vector<geometry_msgs::Point> profile;

    double start_angle = odom_yaw_;
    double start_x = odom_pose_.x;
    double start_y = odom_pose_.y;

    if (move_type == open_loop_move::OpenLoopMoveRequest::DISTANCE)
    {
        trapezoidal_profile(distance, profile, max_vel_linear_, max_accel_linear_, dt_);
    }
    else if (move_type == open_loop_move::OpenLoopMoveRequest::ANGLE)
    {
        trapezoidal_profile(distance, profile, max_vel_angular_, max_accel_angular_, dt_);
    }
    else
    {
        ROS_WARN("Unknown move type %d. Exiting", move_type);
        response.success = false;
        return false;
    }

    ros::Rate timer(1 / dt_);
    
    for (geometry_msgs::Point p : profile)
    {
        if (publish_profile_)
        {
            profile_pub_.publish(p);
        }

        // Control robot to move
        if (move_type == open_loop_move::OpenLoopMoveRequest::DISTANCE)
        {
            straight_path_controller(start_x, start_y, p.x, p.y, p.z);
        }
        else // Both other types are turning
        {
            turn_in_place_controller(start_angle, p.x, p.y, p.z);
        }

        timer.sleep();
        ros::spinOnce();

        // Have to use isShuttingDown in service
        // not ros::ok
        if (ros::isShuttingDown())
        {
            // ros info doesn't seem to publish before node dies
            ROS_INFO("Move interrupted by shutdown");
            std::cout << "Move interrupted by shutdown" << std::endl;
            response.success = false;
            return false;
        }
    }

    ROS_INFO("Done move");

    response.success = true;
    return true;
}

void OpenLoopMove::straight_path_controller(double start_x, double start_y, double target_pos, double target_vel, double target_accel)
{
    double distance_traveled = sqrt((odom_pose_.x - start_x) * (odom_pose_.x - start_x) + (odom_pose_.y - start_y) * (odom_pose_.y - start_y));
    double pos_error = target_pos - distance_traveled;
    double vel_error = target_vel - odom_twist_.linear.x;

    double cmd_vel = target_vel + pos_error * kP_linear_ + vel_error * kV_linear_ + target_accel * kA_linear_;

    geometry_msgs::Twist cmd_vel_msg;
    cmd_vel_msg.linear.x = cmd_vel;
    cmd_vel_pub_.publish(cmd_vel_msg);
}

void OpenLoopMove::turn_in_place_controller(double start_angle, double target_pos, double target_vel, double target_accel)
{
    double angle_error = min_angle(target_pos - (odom_yaw_ - start_angle));
    double vel_error = target_vel - odom_twist_.angular.z;

    double cmd_vel = target_vel + angle_error * kP_angular_ + vel_error * kV_angular_ + target_accel * kA_angular_;

    geometry_msgs::Twist cmd_vel_msg;
    cmd_vel_msg.angular.z = cmd_vel;
    cmd_vel_pub_.publish(cmd_vel_msg);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "open_loop_mover");
    ros::NodeHandle nh("");
    ros::NodeHandle pnh("~");

    OpenLoopMove mover(nh, pnh);
    ros::spin();   
    return 0;
}