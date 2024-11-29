/**
 * @file map_to_odom.h
 * @author 李东权 (1327165187@qq.com)
 * @brief 定位初始值偏移量，重定位接口
 * @version 2.0
 * @date 2024-04-03
 *
 * @copyright Copyright (c) 2024
 *
 */
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
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include "Eigen/Core"
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

class map_to_odom
{
private:
    ros::NodeHandle nh;                               // 句柄
    ros::NodeHandle nhPrivate = ros::NodeHandle("~"); // 参数句柄
    ros::Subscriber subReLocal;                       // 订阅重定位功能包发来的位姿信息
    ros::Subscriber subInitOdom;                      // 订阅重定位功能包发来的位姿信息
    ros::Subscriber subReceiveData;                   // 订阅串口接收到的消息
    ros::Subscriber subStop;                          // 订阅车强制停止
    ros::Subscriber subOdom;                          // 订阅里程计信息
    ros::Publisher pubVehicleToMapPose;               // 发布想要的vehicle到map的坐标变换
    ros::Publisher pubOdomToMapPose;                  // 发布想要的odom到map的坐标变换
    ros::Publisher pubvehicleToOdom;                  // 发布想要的vehicle到odom的坐标变换
    ros::Publisher pubRelocal;
    ros::Publisher pubGoalPoint;
    tf::StampedTransform transform; // 雷达到机器人的静态变换 tf
    tf::TransformListener listener; // tf监听器

    double defaultX = 0;
    double defaultY = 0;
    double defaultYaw = 0;
    geometry_msgs::Quaternion geoQuat_odom;
    int get_relocal_num = 0;
    bool tranf_odom = false;
    geometry_msgs::PoseStamped wantVehicleToMap;
    geometry_msgs::PoseStamped countOdomToMap;
    geometry_msgs::PoseStamped vehicleToOdom;
    int restart_mapping_last = 0;
    bool safetyStop;
    std::string ns;
    Eigen::Isometry3d actu_odom;
    bool get_odom = false;

public:
    map_to_odom();
    ~map_to_odom() = default;
    geometry_msgs::PoseStamped map_to_odom_trans;
    std::string odom_frame;
    std::string vehicle_frame;
    void reLocalizationCallBack(const geometry_msgs::PoseStamped::ConstPtr &vTm_msg);
    void initOdomCallBack(const std_msgs::Bool::ConstPtr &initOdom_msg);
    void stopCallBack(const std_msgs::Bool::ConstPtr &stop);
    void odomCallBack(const nav_msgs::Odometry::ConstPtr &msg);
};