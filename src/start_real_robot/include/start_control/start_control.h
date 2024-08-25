#pragma once
#include "msg_process/receive_data.h"
#include <deque>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>

class StartControl
{
private:
    ros::NodeHandle nh;          // 句柄
    ros::Subscriber serial_sub;  // 订阅 串口信息
    ros::Subscriber rplidar_sub; // 订阅 2D雷达
    ros::Subscriber rplidar_status_sub;

public:
    std_msgs::Bool rplidar_status;
    pcl::PointCloud<pcl::PointXYZI>::Ptr lidar_points;
    StartControl();
    ~StartControl() = default;
    void rplidarCallBack(const sensor_msgs::PointCloud2ConstPtr &addPoints);
    void rplidarStatusCallBack(const std_msgs::BoolConstPtr &status);
};