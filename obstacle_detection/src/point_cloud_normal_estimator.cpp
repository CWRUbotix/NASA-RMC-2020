#include <ros/ros.h>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>

class PointCloudNormalEstimator
{
    public:
        PointCloudNormalEstimator(ros::NodeHandle *nh, ros::NodeHandle *pnh) : nh_(*nh), pnh_(*pnh)
        {
            ROS_INFO("Node initialized");

            viewer_.setBackgroundColor(0.0, 0.0, 0.5);
            viewer_.addCoordinateSystem(0.5, 0, 0, 0);

            pnh_.param("search_radius", search_radius_, 0.1);
            pnh_.param("normal_threshold", normal_threshold_, 0.7);

            ROS_INFO_STREAM("search radius set to " << search_radius_);
            ROS_INFO_STREAM("normal threshold set to " << normal_threshold_);

            cloud_pub_ = nh_.advertise<pcl::PCLPointCloud2>("output_cloud", 1);
            cloud_sub_ = nh_.subscribe("input_cloud", 1, &PointCloudNormalEstimator::receivePointCloud, this);

            viewer_timer_ = nh_.createTimer(ros::Duration(0.1), &PointCloudNormalEstimator::viewerTimerCB, this);

            has_update_ = false;
        }

        // Receives the point cloud message and calculates the normals
        void receivePointCloud(const sensor_msgs::PointCloud2ConstPtr& msg)
        {
            // ROS_INFO_STREAM("Received cloud of size " << cloud->size());
            cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::fromROSMsg(*msg, *cloud_);

            ROS_INFO_STREAM("Cloud size: " << cloud_->size());

            // Create the normal estimation class, and pass the input dataset to it
            pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
            ne.setInputCloud(cloud_);

            pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
            ne.setSearchMethod(tree);

            ne.setRadiusSearch(search_radius_);
            
            cloud_normals_.reset(new pcl::PointCloud<pcl::Normal>);
            ne.compute(*cloud_normals_);

            has_update_ = true;
        }

        void shutdown()
        {
            viewer_.close();
        }

        inline float packRGB(uint8_t r, uint8_t g, uint8_t b)
        {
          uint32_t color_uint = ((uint32_t)r << 16 | (uint32_t) g << 8 | (uint32_t)b);
          return *reinterpret_cast<float*>(&color_uint);
        }

        // Handles drawing the pointcloud in the viewer.
        // Colors the pointcloud based on whether a point
        // Is determined to be an obstacle or not
        void viewerTimerCB(const ros::TimerEvent& event)
        {
            if(has_update_)
            {
                has_update_ = false;

                pcl::PointCloud<pcl::PointXYZRGB>::Ptr color_cloud(new pcl::PointCloud<pcl::PointXYZRGB>()); //your cloud that you want to colorize

                color_cloud->width = cloud_->size();
                color_cloud->height = 1;
                color_cloud->points.resize(color_cloud->width * color_cloud->height);

                  for(int i = 0; i < cloud_->size(); ++i){
                    color_cloud->points[i].x = cloud_->points[i].x;
                    color_cloud->points[i].y = cloud_->points[i].y;
                    color_cloud->points[i].z = cloud_->points[i].z;

                    float color_value = fabs(cloud_normals_->points[i].normal_z) < normal_threshold_ ? packRGB(255, 0, 0) : packRGB(0, 255, 0);
                    color_cloud->points[i].rgb = color_value;
                  }

                pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> cloud_color_handle(color_cloud);

                if (!viewer_.contains("cloud"))
                {
                    viewer_.addPointCloud<pcl::PointXYZRGB>(color_cloud, cloud_color_handle);

                } else {
                    viewer_.updatePointCloud<pcl::PointXYZRGB>(color_cloud, cloud_color_handle);
                    viewer_.removePointCloud("cloud_normals", 0);
                }

                viewer_.addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(color_cloud, cloud_normals_, 1, 0.03, "cloud_normals");

                viewer_.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "cloud");
            }

            viewer_.spinOnce();

            if(viewer_.wasStopped())
            {
                ros::shutdown();
            }
        }

    private:
        ros::NodeHandle nh_;
        ros::NodeHandle pnh_;

        ros::Publisher cloud_pub_;
        ros::Subscriber cloud_sub_;

        pcl::visualization::PCLVisualizer viewer_;  // The pcl viewer
        bool has_update_;  // Point cloud has been updated
        ros::Timer viewer_timer_;  // Reference to timer

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;  // The cloud itself
        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals_;  // The cloud's normals

        double search_radius_;  // Normal estimation parameter
        double normal_threshold_;  // When a point is determined non-ground
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "point_cloud_normal_estimator");

    ros::NodeHandle nh("");
    ros::NodeHandle pnh("~");

    PointCloudNormalEstimator pcl_normal_estimator(&nh, &pnh);
    ros::spin();
}
