#pragma once
#include <deque>
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

public:
    StartControl();
    ~StartControl() = default;
};