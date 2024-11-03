#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/sac_model_cone.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>

ros::Publisher pub;

void coneReconstruction(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    // 將ROS PointCloud2消息轉換為PCL PointCloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*cloud_msg, *cloud);

    // 使用 RANSAC 進行圓錐體的分割
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZ> seg;

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_CIRCLE3D);
    seg.setMethodType(pcl::SAC_RANSAC);

    // 設定SAC參數
    seg.setMaxIterations(10000000);  // 根據需求調整
    seg.setDistanceThreshold(0.1);  // 根據需求調整

    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);

    // 提取屬於圓錐體的點雲
    pcl::PointCloud<pcl::PointXYZ>::Ptr cone_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.filter(*cone_cloud);

    // 將新的點雲合併到先前的所有點雲
    static pcl::PointCloud<pcl::PointXYZ> all_cones;
    all_cones += *cone_cloud;

    // 發佈所有合併的點雲
    sensor_msgs::PointCloud2 all_cones_msg;
    pcl::toROSMsg(all_cones, all_cones_msg);
    all_cones_msg.header = cloud_msg->header;
    pub.publish(all_cones_msg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "cone_reconstruction");
    ros::NodeHandle nh;

    // 訂閱輸入的PointCloud2話題
    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/clustered_cloud", 1, coneReconstruction);

    // 廣播重建後的所有錐體的PointCloud2話題
    pub = nh.advertise<sensor_msgs::PointCloud2>("/reconstructed_cones", 1);

    ros::spin();

    return 0;
}
