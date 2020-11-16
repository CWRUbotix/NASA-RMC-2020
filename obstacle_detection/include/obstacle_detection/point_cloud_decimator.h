#ifndef POINT_CLOUD_DECIMATOR_H
#define POINT_CLOUD_DECIMATOR_H

#include <ros/ros.h>

#include <nodelet/nodelet.h>

#include <pcl_conversions/pcl_conversions.h>


namespace obstacle_detection
{

class PointCloudDecimator : public nodelet::Nodelet
{
    public:
        // Nodelet init function
        virtual void onInit();

        // Point cloud callback
        void receive_point_cloud(const pcl::PCLPointCloud2ConstPtr& cloud);

    private:
        // Nodehandles
        ros::NodeHandle nh_;
        ros::NodeHandle pnh_;

        // Pubs and subs
        ros::Publisher cloud_pub_;
        ros::Subscriber cloud_sub_;

        // Filter leaf size
        double voxel_size_;
};

}  // namespace obstacle_detection

#endif  // POINT_CLOUD_DECIMATOR_H
