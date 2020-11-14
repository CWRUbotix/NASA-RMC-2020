#include <ros/ros.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/filters/voxel_grid.h>

ros::Publisher pub;

void receive_point_cloud(const pcl::PCLPointCloud2ConstPtr& cloud)
{
  pcl::PCLPointCloud2 cloud_filtered;

  // Perform the actual filtering
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud(cloud);
  sor.setLeafSize(0.05, 0.05, 0.05);
  sor.filter(cloud_filtered);

  // Publish the data
  pub.publish(cloud_filtered);
}

int main(int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "point_cloud_decimator");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("input_cloud", 1, receive_point_cloud);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<pcl::PCLPointCloud2> ("output_cloud", 1);

  // Spin
  ros::spin();
}