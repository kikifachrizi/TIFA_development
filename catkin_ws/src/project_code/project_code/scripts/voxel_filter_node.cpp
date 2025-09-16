#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>

ros::Publisher pub;

void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& input)
{
    // Konversi dari ROS PointCloud2 ke PCL PointCloud
    pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2);
    pcl_conversions::toPCL(*input, *cloud);

    pcl::PCLPointCloud2 cloud_filtered;
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(0.1, 0.1, 0.1);
    sor.filter(cloud_filtered);

    // Konversi kembali ke ROS PointCloud2
    sensor_msgs::PointCloud2 output;
    pcl_conversions::fromPCL(cloud_filtered, output);

    // Publish hasil filter
    pub.publish(output);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "voxel_filter_node");
    ros::NodeHandle nh;

    // Subscribing ke topic PointCloud input
    ros::Subscriber sub = nh.subscribe("/scan", 1, cloudCallback);
    
    // Publishing hasil filter
    pub = nh.advertise<sensor_msgs::PointCloud2>("output_cloud", 1);

    ros::spin();
}
