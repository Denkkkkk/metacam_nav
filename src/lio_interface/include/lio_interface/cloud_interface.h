#pragma once
#include <math.h>
#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

class CloudInterface
{
    ros::NodeHandle nh;                               // 句柄
    ros::NodeHandle nhPrivate = ros::NodeHandle("~"); // 私有参数句柄
    const double PI = 3.1415926;
    tf::TransformListener listener;
    ros::Subscriber subScan;
    ros::Subscriber subOdom;
    ros::Subscriber subSpeed;
    ros::Publisher pubspeed;
    ros::Publisher pubScan;
    ros::Subscriber subOdometry;
    std::string robotFrame;
    float vehicleRoll = 0, vehiclePitch = 0, vehicleYaw = 0;
    float vehicleX = 0, vehicleY = 0, vehicleZ = 0;
    std::string ns;
    std::string cloud_in_topic;
    std::string cloud_out_topic;
    std::string odom_topic;
    bool get_odom = false;
    std_msgs::Float32 speed;
    double blind_range;

    void scanHandler(const sensor_msgs::PointCloud2::ConstPtr &scanIn);
    void odomHandler(const nav_msgs::Odometry::ConstPtr &odom);
    void speedHandler(const std_msgs::Float32::ConstPtr &msg);

public:
    CloudInterface();
    ~CloudInterface() = default;
};