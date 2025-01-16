#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>

ros::Publisher pub;

// 將 PCLVisualizer 的初始化移除
// pcl::visualization::PCLVisualizer viewer("PCL Viewer");

void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*cloud_msg, *cloud);

    // 定义视野修剪框的范围
    pcl::CropBox<pcl::PointXYZ> crop;
    crop.setInputCloud(cloud);
    crop.setMin(Eigen::Vector4f(-3.0, 0.0, -1.0, 1.0));  // 定义修剪框的最小范围
    crop.setMax(Eigen::Vector4f(3.0, 15.0, 2.0, 1.0));     // 定义修剪框的最大范围
    pcl::PointCloud<pcl::PointXYZ> cropped_cloud;
    crop.filter(cropped_cloud);

    // 发布修剪后的点云
    sensor_msgs::PointCloud2 output_cloud;
    pcl::toROSMsg(cropped_cloud, output_cloud);
    output_cloud.header = cloud_msg->header;
    pub.publish(output_cloud);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "conditional_removal");
    ros::NodeHandle nh;

    // 订阅原始点云主题
    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/rslidar_points", 1, cloudCallback);

    // 创建点云发布者
    pub = nh.advertise<sensor_msgs::PointCloud2>("/field_of_view_trimming", 1);

    // 开始 ROS 循环
    ros::spin();

    return 0;
}
