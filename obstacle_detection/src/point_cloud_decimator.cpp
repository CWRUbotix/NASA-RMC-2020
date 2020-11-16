#include <ros/ros.h>

#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/filters/voxel_grid.h>

namespace obstacle_detection
{

class PointCloudDecimator : public nodelet::Nodelet
{
    public:
        virtual void onInit()
        {
            nh_ = getNodeHandle();
            pnh_ = getPrivateNodeHandle();

            pnh_.param("voxel_size", voxel_size_, 0.05);
            NODELET_INFO_STREAM("voxel_size set to " << voxel_size_);

            cloud_pub_ = nh_.advertise<pcl::PCLPointCloud2>("output_cloud", 1);
            sub_ = nh_.subscribe("input_cloud", 1, &PointCloudDecimator::receive_point_cloud, this);
        }

        void receive_point_cloud(const pcl::PCLPointCloud2ConstPtr& cloud)
        {
          pcl::PCLPointCloud2 cloud_filtered;

          // Perform the actual filtering
          pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
          sor.setInputCloud(cloud);
          sor.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
          sor.filter(cloud_filtered);

          // Publish the data
          cloud_pub_.publish(cloud_filtered);
        }

    private:
        ros::NodeHandle nh_;
        ros::NodeHandle pnh_;

        ros::Publisher cloud_pub_;
        ros::Subscriber sub_;

        double voxel_size_;
};

}  // namespace obstacle_detection

// Register with pluginlib
PLUGINLIB_EXPORT_CLASS(obstacle_detection::PointCloudDecimator, nodelet::Nodelet)
