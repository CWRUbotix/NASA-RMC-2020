#include <ros/ros.h>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/console/time.h>   // TicToc

#include <geometry_msgs/PoseWithCovarianceStamped.h>

class PointCloudICPOdometry
{
    public:
        PointCloudICPOdometry(ros::NodeHandle *nh, ros::NodeHandle *pnh) : nh_(*nh), pnh_(*pnh)
        {
            ROS_INFO("ICP odometry node initialized");

            pnh_.param("visualize", visualize_, true);
            pnh_.param("max_iterations", max_iterations_, 30);

            ROS_INFO_STREAM("max_iterations set to " << max_iterations_);

            odom_pub_ =  nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("odometry_out", 1);
            cloud_sub_ = nh_.subscribe("input_cloud", 1, &PointCloudICPOdometry::receivePointCloud, this);

            if (visualize_) {
                viewer_.reset(new pcl::visualization::PCLVisualizer);
                viewer_->setBackgroundColor(0.0, 0.0, 0.5);
                viewer_->addCoordinateSystem(0.5, 0, 0, 0);

                viewer_timer_ = nh_.createTimer(ros::Duration(0.1), &PointCloudICPOdometry::viewerTimerCB, this);
            }

            total_odom_ = Eigen::Matrix4d::Identity();

            has_update_ = false;
        }

        // Receives the point cloud message and calculates the normals
        void receivePointCloud(const sensor_msgs::PointCloud2ConstPtr& msg)
        {
            count_++;
            if (count_ % 1 != 0){
                return;
            }
            // Copy and store cloud
            if (cloud_){
                last_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>(*cloud_));
            }

            cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::fromROSMsg(*msg, *cloud_);

            if (!last_cloud_){
                return;
            }

            pcl::console::TicToc time;
            time.tic ();
            pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
            icp.setMaximumIterations(max_iterations_);
            icp.setInputSource(last_cloud_);
            icp.setInputTarget(cloud_);
            icp.align(*last_cloud_);
            std::cout << "Applied " << max_iterations_ << " ICP iteration(s) in " << time.toc() << " ms" << std::endl;

            Eigen::Matrix4d transformation_matrix;
            if (icp.hasConverged ())
            {
                std::cout << "\nICP has converged, score is " << icp.getFitnessScore () << std::endl;
                transformation_matrix = icp.getFinalTransformation().cast<double>();
                print4x4Matrix(transformation_matrix);
            }
            else
            {
                PCL_ERROR("\nICP has not converged.\n");
            }

            total_odom_ = transformation_matrix * total_odom_;

            geometry_msgs::PoseWithCovarianceStamped pose_out;
            matrix4x4ToPose(total_odom_, pose_out);
            pose_out.header = msg->header;
            pose_out.header.frame_id = "map";
            odom_pub_.publish(pose_out);

            has_update_ = true;
        }

        void shutdown()
        {
            viewer_->close();
        }

        inline float packRGB(uint8_t r, uint8_t g, uint8_t b)
        {
          uint32_t color_uint = ((uint32_t)r << 16 | (uint32_t) g << 8 | (uint32_t)b);
          return *reinterpret_cast<float*>(&color_uint);
        }

        void print4x4Matrix (const Eigen::Matrix4d & matrix)
        {
          printf ("Rotation matrix :\n");
          printf ("    | %6.3f %6.3f %6.3f | \n", matrix (0, 0), matrix (0, 1), matrix (0, 2));
          printf ("R = | %6.3f %6.3f %6.3f | \n", matrix (1, 0), matrix (1, 1), matrix (1, 2));
          printf ("    | %6.3f %6.3f %6.3f | \n", matrix (2, 0), matrix (2, 1), matrix (2, 2));
          printf ("Translation vector :\n");
          printf ("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix (0, 3), matrix (1, 3), matrix (2, 3));
        }

        void matrix4x4ToPose(const Eigen::Matrix4d &matrix, geometry_msgs::PoseWithCovarianceStamped &pose)
        {
            Eigen::Matrix3d rot;
            rot(0, 0) = matrix(0, 0); rot(0, 1) = matrix(0, 1); rot(0, 2) = matrix(0, 2);
            rot(1, 0) = matrix(1, 0); rot(1, 1) = matrix(1, 1); rot(1, 2) = matrix(1, 2);
            rot(2, 0) = matrix(2, 0); rot(2, 1) = matrix(2, 1); rot(2, 2) = matrix(2, 2);
            Eigen::Quaterniond rot_quat(rot);

            pose.pose.pose.position.x = matrix(0, 3);
            pose.pose.pose.position.y = matrix(1, 3);
            pose.pose.pose.position.z = matrix(2, 3);

            pose.pose.pose.orientation.x = rot_quat.x();
            pose.pose.pose.orientation.y = rot_quat.y();
            pose.pose.pose.orientation.z = rot_quat.z();
            pose.pose.pose.orientation.w = rot_quat.w();
        }

        // Handles drawing the pointcloud in the viewer.
        // Colors the pointcloud based on whether a point
        // Is determined to be an obstacle or not
        void viewerTimerCB(const ros::TimerEvent& event)
        {
            if(has_update_)
            {
                has_update_ = false;

                pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color_cloud(cloud_, 255, 255, 255);
                pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color_last_cloud(last_cloud_, 255, 100, 255);

                if (!viewer_->contains("cloud"))
                {
                    viewer_->addPointCloud<pcl::PointXYZ>(cloud_, single_color_cloud, "cloud");
                } else {
                    viewer_->updatePointCloud<pcl::PointXYZ>(cloud_, single_color_cloud, "cloud");
                }

                if (!viewer_->contains("last_cloud"))
                {
                    viewer_->addPointCloud<pcl::PointXYZ>(last_cloud_, single_color_last_cloud, "last_cloud");
                } else {
                    viewer_->updatePointCloud<pcl::PointXYZ>(last_cloud_, single_color_last_cloud, "last_cloud");
                }

                viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "cloud");
                viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "last_cloud");
            }

            viewer_->spinOnce();

            if(viewer_->wasStopped())
            {
                ros::shutdown();
            }
        }

    private:
        ros::NodeHandle nh_;
        ros::NodeHandle pnh_;

        ros::Publisher odom_pub_; // Calculated odometry pub
        ros::Subscriber cloud_sub_;

        pcl::visualization::PCLVisualizer::Ptr viewer_;  // The pcl viewer
        bool has_update_;  // Point cloud has been updated
        ros::Timer viewer_timer_;  // Reference to timer

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;  // The current cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr last_cloud_;  // The last recieved cloud

        Eigen::Matrix4d total_odom_;  // Cumulative odom

        int max_iterations_;  // Maximum number of icp iterations;
        bool visualize_;  // Whether or not to display viewer

        int count_;  // Only do icp every other time
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "point_cloud_icp_odometry");

    ros::NodeHandle nh("");
    ros::NodeHandle pnh("~");

    PointCloudICPOdometry pcl_icp_odometry(&nh, &pnh);
    ros::spin();
}
