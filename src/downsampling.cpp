#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>

ros::Publisher pub;

void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    // Convert ROS PointCloud2 message to PCL PointCloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*cloud_msg, *cloud);

    // 移除點雲中的 NaN 值
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud, *cloud_filtered, indices);
    
    //篩選點雲中三個維度的數據
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud_filtered);
    pass.setFilterFieldName("x"); // 過濾 X 軸
    pass.setFilterLimits(-100.0, 100.0); // 設定合理的範圍
    pass.filter(*cloud_filtered);

    pass.setFilterFieldName("y"); // 過濾 Y 軸
    pass.setFilterLimits(-100.0, 100.0);
    pass.filter(*cloud_filtered);

    pass.setFilterFieldName("z"); // 過濾 Z 軸
    pass.setFilterLimits(0.0, 10.0); // 假設 Z 軸應該在此範圍
    pass.filter(*cloud_filtered);

    // Voxel grid downsampling
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(cloud_filtered);
    vg.setLeafSize(0.02, 0.02, 0.02);  // Adjust the leaf size according to your needs
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downsampled(new pcl::PointCloud<pcl::PointXYZ>);
    vg.filter(*cloud_downsampled);

    // Publish the combined cluster as a single PointCloud2 message
    sensor_msgs::PointCloud2 downsampled_msg;
    pcl::toROSMsg(*cloud_downsampled, downsampled_msg);
    pub.publish(downsampled_msg);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "euclidean_clustering");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/rslidar_points", 1, cloudCallback);

    pub = nh.advertise<sensor_msgs::PointCloud2>("/downsampled_cloud", 1);

    ros::spin();
    
    return 0;
}