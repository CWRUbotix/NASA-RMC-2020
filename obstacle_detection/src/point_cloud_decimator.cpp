#include "obstacle_detection/point_cloud_decimator.h"

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>


namespace obstacle_detection
{

void PointCloudDecimator::onInit()
{
    nh_ = getNodeHandle();
    pnh_ = getPrivateNodeHandle();

    pnh_.param("voxel_size", voxel_size_, 0.05);
    NODELET_INFO_STREAM("voxel_size set to " << voxel_size_);

    tf_listener_ = new tf2_ros::TransformListener(tf_buffer_);

    cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("output_cloud", 1);
    cloud_sub_ = nh_.subscribe("input_cloud", 1, &PointCloudDecimator::receive_point_cloud, this);
}

void PointCloudDecimator::receive_point_cloud(const pcl::PCLPointCloud2ConstPtr& cloud)
{
  pcl::PCLPointCloud2 cloud_filtered;

  // Perform the actual filtering
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud(cloud);
  sor.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
  sor.filter(cloud_filtered);

  sensor_msgs::PointCloud2 cloud_filtered_2, out_cloud;
  pcl_conversions::moveFromPCL(cloud_filtered, cloud_filtered_2);

  try {
    tf_buffer_.transform(cloud_filtered_2, out_cloud, "base_link", ros::Duration(0.1));
  } catch (tf2::TransformException &ex){
    ROS_WARN("%s", ex.what());
    return;
  }

  // Publish the data
  cloud_pub_.publish(out_cloud);
}

}  // namespace obstacle_detection

// Register with pluginlib
PLUGINLIB_EXPORT_CLASS(obstacle_detection::PointCloudDecimator, nodelet::Nodelet)
