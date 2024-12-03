#pragma once
#include <math.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float32.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include "Eigen/Core"
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

class OdomInterface
{
    ros::NodeHandle nh;                               // 句柄
    ros::NodeHandle nhPrivate = ros::NodeHandle("~"); // 私有参数句柄
    ros::Publisher pubOdom;                           // 发布fastlio处理过后的位姿信息发布到cmu
    ros::Subscriber subOdom;                          // 订阅livox雷达发来的位姿信息
    tf::TransformBroadcaster tfBroadcaster;           // tf坐标变换广播器
    tf::StampedTransform odomTrans;                   // tf坐标变换
    nav_msgs::Odometry odomData;                      // 最终发布的里程计数据
    tf::StampedTransform transform;                   // 雷达到机器人的静态变换 tf
    tf::TransformListener listener;                   // tf监听器

    float vehicleX = 0;
    float vehicleY = 0;
    float vehicleZ = 0;
    float vehicleRoll = 0;
    float vehiclePitch = 0;
    float vehicleYaw = 0;
    geometry_msgs::Quaternion geoQuat_odom;
    int get_odom_num = 0;
    bool tranf_odom = false;
    std::string ns;
    std::string odom_in_topic;
    std::string odom_out_topic;
    std::string lidar_frame;
    double rate_x;
    double rate_y;

public:
    OdomInterface();
    ~OdomInterface() = default;
    /**
     * 订阅livox雷达位姿信息的回调函数
     * @param odom
     */
    void odomCallBack(const nav_msgs::Odometry::ConstPtr &odom);
};