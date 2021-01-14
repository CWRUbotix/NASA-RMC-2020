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

    publish_profile_ = true;
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
    else if (move_type == open_loop_move::OpenLoopMoveRequest::TO_ANGLE)
    {
        return "To angle";
    }
    else if (move_type == open_loop_move::OpenLoopMoveRequest::FOR_ANGLE)
    {
        return "For angle";
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

    trapezoidal_profile(distance, profile);

    ros::Rate timer(1 / DT);
    
    for (geometry_msgs::Point p : profile)
    {
        profile_pub_.publish(p);

        // Control robot to move
        if (move_type == open_loop_move::OpenLoopMoveRequest::DISTANCE)
        {
            straight_path_controller(p.x, p.y, p.z);
        }
        else // Both other types are turning
        {
            turn_in_place_controller(p.x, p.y, p.z);
        }

        timer.sleep();

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

void OpenLoopMove::straight_path_controller(double pos, double vel, double accel)
{
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = vel;

    cmd_vel_pub_.publish(cmd_vel);
}

void OpenLoopMove::turn_in_place_controller(double pos, double vel, double accel)
{
    geometry_msgs::Twist cmd_vel;
    cmd_vel.angular.z = vel;

    cmd_vel_pub_.publish(cmd_vel);
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