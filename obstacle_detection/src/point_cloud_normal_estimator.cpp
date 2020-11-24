#include <ros/ros.h>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>


class PointCloudNormalEstimator
{
    public:
        PointCloudNormalEstimator(ros::NodeHandle *nh, ros::NodeHandle *pnh) : nh_(*nh), pnh_(*pnh)
        {
            ROS_INFO("Node initialized");

            pnh_.param("search_radius", search_radius_, 0.1);
            pnh_.param("normal_threshold", normal_threshold_, 0.7);
            pnh_.param("cluster_tolerance", cluster_tolerance_, 0.1);
            pnh_.param("visualize", visualize_, true);

            ROS_INFO_STREAM("search radius set to " << search_radius_);
            ROS_INFO_STREAM("normal threshold set to " << normal_threshold_);
            ROS_INFO_STREAM("cluster_tolerance set to " << cluster_tolerance_);
            ROS_INFO_STREAM("visualize set to " << visualize_);

            cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("output_cloud", 1);
            cloud_sub_ = nh_.subscribe("input_cloud", 1, &PointCloudNormalEstimator::receivePointCloud, this);

            if (visualize_) {
                viewer_.reset(new pcl::visualization::PCLVisualizer);
                viewer_->setBackgroundColor(0.0, 0.0, 0.5);
                viewer_->addCoordinateSystem(0.5, 0, 0, 0);

                viewer_timer_ = nh_.createTimer(ros::Duration(0.1), &PointCloudNormalEstimator::viewerTimerCB, this);
            }

            has_update_ = false;
        }

        // Receives the point cloud message and calculates the normals
        void receivePointCloud(const sensor_msgs::PointCloud2ConstPtr& msg)
        {
            cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::fromROSMsg(*msg, *cloud_);

            // Create the normal estimation class, and pass the input dataset to it
            pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
            ne.setInputCloud(cloud_);

            pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
            ne.setSearchMethod(tree);

            ne.setRadiusSearch(search_radius_);
            
            cloud_normals_.reset(new pcl::PointCloud<pcl::Normal>);
            ne.compute(*cloud_normals_);

            pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            sensor_msgs::PointCloud2 out_cloud_ros;

            // Find obstacle points
            extractObstaclesFromNormals(cloud_normals_, cloud_, out_cloud);

            // Create new tree for euclidean clustering
            tree.reset(new pcl::search::KdTree<pcl::PointXYZ>);
            tree->setInputCloud(out_cloud);

            // Extract the clusters
            std::vector<pcl::PointIndices> cluster_indices;
            pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
            ec.setClusterTolerance(cluster_tolerance_);
            ec.setMinClusterSize(10);
            ec.setMaxClusterSize(10000);
            ec.setSearchMethod(tree);
            ec.setInputCloud(out_cloud);
            ec.extract(cluster_indices);

            // Create colored cloud for display purposes
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cloud_colored(new pcl::PointCloud<pcl::PointXYZRGB>);

            // Extract the clusters into a new cloud and color them
            createColoredClusterCloud(out_cloud, cluster_indices, out_cloud_colored);

            pcl::toROSMsg(*out_cloud_colored, out_cloud_ros);
            out_cloud_ros.header = msg->header;

            cloud_pub_.publish(out_cloud_ros);

            has_update_ = true;
        }

        void extractObstaclesFromNormals(pcl::PointCloud<pcl::Normal>::Ptr normals, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                         pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud)
        {
            // Find those with a non-upward pointing normal vector
            std::vector<int> indices;
            for(int i = 0; i < cloud_->size(); ++i){
                if (fabs(cloud_normals_->points[i].normal_z) < normal_threshold_)
                {
                    indices.push_back(i);
                }
            }

            pcl::IndicesPtr obstacle_indices = pcl::IndicesPtr(new std::vector<int>(indices));

            // Extract points from cloud
            pcl::ExtractIndices<pcl::PointXYZ> extract;
            extract.setInputCloud(cloud);
            extract.setIndices(obstacle_indices);
            extract.setNegative(false);
            extract.filter(*out_cloud);
        }

        void createColoredClusterCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const std::vector<pcl::PointIndices> &clusters,
                                       pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_colored)
        {
            // Modify colors of cloud to display different clusters
            int j = 0;
            float color_palette[10];
            for(int i = 0; i < 10; ++i)
            {
                color_palette[i] = 4324.98213 * (i*i + 1);  // Random colors
            }

            obstacle_models_.clear();

            // Iterate over clusters
            for (std::vector<pcl::PointIndices>::const_iterator it = clusters.begin(); it != clusters.end(); ++it)
            {
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster_colored(new pcl::PointCloud<pcl::PointXYZRGB>);

                // Extract a cluster's points
                pcl::ExtractIndices<pcl::PointXYZ> extract;
                extract.setInputCloud(cloud);
                extract.setIndices(pcl::IndicesPtr(new std::vector<int>((*it).indices)));
                extract.filter(*cloud_cluster);

                // Find sphere that models points
                pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
                pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
                // Create the segmentation object
                pcl::SACSegmentation<pcl::PointXYZ> seg;
                // Optional
                seg.setOptimizeCoefficients(false);
                // Mandatory
                seg.setModelType(pcl::SACMODEL_SPHERE);
                seg.setMethodType(pcl::SAC_RANSAC);
                seg.setDistanceThreshold(0.01);
                seg.setRadiusLimits(0.1, 0.3);
                seg.setInputCloud(cloud_cluster);
                seg.segment(*inliers, *coefficients);

                if (inliers->indices.size() != 0)
                {
                    obstacle_models_.push_back(*coefficients);
                }

                pcl::copyPointCloud(*cloud_cluster, *cloud_cluster_colored);

                // Color those points
                for(pcl::PointXYZRGB &point : *cloud_cluster_colored)
                {
                    point.rgb = color_palette[j];
                }

                // Add them to the main cloud
                *cloud_colored += *cloud_cluster_colored;

                j++;
            }
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

                if (!viewer_->contains("cloud"))
                {
                    viewer_->addPointCloud<pcl::PointXYZRGB>(color_cloud, cloud_color_handle);

                } else {
                    viewer_->updatePointCloud<pcl::PointXYZRGB>(color_cloud, cloud_color_handle);
                    viewer_->removePointCloud("cloud_normals", 0);
                }

                viewer_->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(color_cloud, cloud_normals_, 1, 0.03, "cloud_normals");

                viewer_->removeAllShapes();
                for (int i = 0; i < obstacle_models_.size(); ++i)
                {
                    viewer_->addSphere(obstacle_models_[i], "sphere_" + std::to_string(i));
                } 

                viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "cloud");
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

        ros::Publisher cloud_pub_;
        ros::Subscriber cloud_sub_;

        pcl::visualization::PCLVisualizer::Ptr viewer_;  // The pcl viewer
        bool has_update_;  // Point cloud has been updated
        ros::Timer viewer_timer_;  // Reference to timer

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;  // The cloud itself
        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals_;  // The cloud's normals
        pcl::IndicesPtr obstacle_indices_;  // Indices of found obstacles
        std::vector<pcl::ModelCoefficients> obstacle_models_;  // Sphere model parameters for each obstacle  

        double search_radius_;  // Normal estimation parameter
        double normal_threshold_;  // When a point is determined non-ground
        double cluster_tolerance_;  // Euclidean clustering tolerance
        bool visualize_;  // Whether or not to display viewer

};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "point_cloud_normal_estimator");

    ros::NodeHandle nh("");
    ros::NodeHandle pnh("~");

    PointCloudNormalEstimator pcl_normal_estimator(&nh, &pnh);
    ros::spin();
}
