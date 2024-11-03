#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/kdtree/kdtree.h>

ros::Publisher pub;

void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    // Convert ROS PointCloud2 message to PCL PointCloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*cloud_msg, *cloud);

    // Voxel grid downsampling
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(cloud);
    vg.setLeafSize(0.05, 0.05, 0.05);  // Adjust the leaf size according to your needs
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downsampled(new pcl::PointCloud<pcl::PointXYZ>);
    vg.filter(*cloud_downsampled);

    // Euclidean clustering
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud_downsampled);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.05);  // Adjust the cluster tolerance according to your needs
    ec.setMinClusterSize(10);     // Adjust the minimum cluster size according to your needs
    ec.setMaxClusterSize(30);   // Adjust the maximum cluster size according to your needs
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_downsampled);
    ec.extract(cluster_indices);

    // Combine all clusters into a single PointCloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
    for (const auto& indices : cluster_indices)
    {
        for (const auto& index : indices.indices)
        {
            pcl::PointXYZRGB point;
            point.x = cloud_downsampled->points[index].x;
            point.y = cloud_downsampled->points[index].y;
            point.z = cloud_downsampled->points[index].z;

            // Assign random color to each point
            point.r = rand() % 256;
            point.g = rand() % 256;
            point.b = rand() % 256;

            colored_cluster->points.push_back(point);
        }
    }

    // Publish the combined cluster as a single PointCloud2 message
    sensor_msgs::PointCloud2 cluster_msg;
    pcl::toROSMsg(*colored_cluster, cluster_msg);
    cluster_msg.header = cloud_msg->header;
    pub.publish(cluster_msg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "euclidean_clustering");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/ground_segmentation/obstacle_cloud", 1, cloudCallback);

    pub = nh.advertise<sensor_msgs::PointCloud2>("/clustered_cloud", 1);

    ros::spin();
    
    return 0;
}