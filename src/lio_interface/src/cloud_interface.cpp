/**
 * @file cloud_interface.cpp
 * @author 李东权 1327165187@qq.com
 * @brief 点云输出中间件
 * @version 0.1
 * @date 2024-04-30
 *
 * @copyright Copyright (c) 2024
 *
 */
#include <lio_interface/cloud_interface.h>

CloudInterface::CloudInterface()
{
    nhPrivate.param<std::string>("robotFrame", robotFrame, "/vehicle");
    nhPrivate.param<std::string>("ns", ns, "");
    cloud_in_topic = ns + "/cloud_registered";
    cloud_out_topic = ns + "/cloud_interface";
    odom_topic = ns + "/odom_interface";

    subScan = nh.subscribe<sensor_msgs::PointCloud2>(cloud_in_topic, 2, &CloudInterface::scanHandler, this);
    subOdometry = nh.subscribe<nav_msgs::Odometry>(odom_topic, 5, &CloudInterface::odomHandler, this);
    // subSpeed = nh.subscribe<std_msgs::Float32>("/speed", 5, &CloudInterface::speedHandler, this);
    pubScan = nh.advertise<sensor_msgs::PointCloud2>(cloud_out_topic, 100000);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cloud_interface");
    CloudInterface cloud_interface;
    ROS_INFO("cloud_interface started.\n\n");
    ros::spin();
    return 0;
}

void CloudInterface::scanHandler(const sensor_msgs::PointCloud2::ConstPtr &scanIn)
{
    if (!get_odom)
    {
        return;
    }
    // 从lio输出的map_init转到map
    sensor_msgs::PointCloud2 scanData_map;
    pcl_ros::transformPointCloud("map", *scanIn, scanData_map, listener);
    scanData_map.header.stamp = scanIn->header.stamp;
    scanData_map.header.frame_id = "map";
    pubScan.publish(scanData_map);
}

void CloudInterface::odomHandler(const nav_msgs::Odometry::ConstPtr &odom)
{
    double roll, pitch, yaw;
    geometry_msgs::Quaternion geoQuat = odom->pose.pose.orientation;
    tf::Matrix3x3(tf::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w)).getRPY(roll, pitch, yaw);
    vehicleRoll = roll;
    vehiclePitch = pitch;
    vehicleYaw = yaw;
    vehicleX = odom->pose.pose.position.x;
    vehicleY = odom->pose.pose.position.y;
    vehicleZ = odom->pose.pose.position.z;
    get_odom = true;
}

void CloudInterface::speedHandler(const std_msgs::Float32::ConstPtr &msg)
{
    speed = *msg;
}
