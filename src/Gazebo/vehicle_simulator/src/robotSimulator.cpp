/**
 * @file robotSimulator.cpp
 * @author 李东权 (1327165187@qq.com)
 * @brief
 * @version 1.0
 * @date 2024-02-22
 *
 * @copyright Copyright (c) 2024
 *
 */
#include <math.h>
#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

/*
- 在仿真中：
订阅：
  激光雷达的点云
  地形的点云
  速度话题
发布：
  cloud registered
  里程计信息
  map到sensor的tf转换odom
对于控制器：/controller/odom 的父坐标系是odom，子坐标系是base_link
*/
using namespace std;
const double PI = 3.1415926;
ros::Publisher *pubScanPointer = NULL;
ros::Publisher *pubRobotSpeed = NULL;
tf::TransformListener *tf_listener_ptr;
ros::Subscriber subScan;
ros::Subscriber subScan2;
ros::Subscriber subOdom;
ros::Subscriber subSpeed;
ros::Publisher pubVehicleOdom;
ros::Publisher pubspeed;
ros::Publisher pubScan;
nav_msgs::Odometry odomData;
std::string ns;
std::string robotFrame;
std::string velodyne_topic1;
std::string velodyne_topic2;
sensor_msgs::PointCloud2 registered_points;
sensor_msgs::PointCloud2 velodyne_points1;
sensor_msgs::PointCloud2 velodyne_points2;

void scanHandler(const sensor_msgs::PointCloud2::ConstPtr &scanIn);
void scan2Handler(const sensor_msgs::PointCloud2::ConstPtr &scanIn);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robotSimulator");
    ros::NodeHandle nh;
    ros::NodeHandle nhPrivate = ros::NodeHandle("~");
    ros::Duration(1).sleep(); // 等待map坐标系初始化完成
    nhPrivate.param<std::string>("ns", ns, "");
    if (ns != "")
    {
        ns += "/";
    }
    robotFrame = ns + "vehicle";
    velodyne_topic1 = ns + "/velodyne_points";
    velodyne_topic2 = ns + "/velodyne_points2";
    tf::TransformListener listener;
    tf_listener_ptr = &listener;
    subScan = nh.subscribe<sensor_msgs::PointCloud2>(velodyne_topic1, 5, scanHandler);
    // subScan2 = nh.subscribe<sensor_msgs::PointCloud2>(velodyne_topic2, 5, scan2Handler);
    pubVehicleOdom = nh.advertise<nav_msgs::Odometry>("/Odometry", 5);
    pubScan = nh.advertise<sensor_msgs::PointCloud2>("/cloud_registered", 5);
    // 使pubscan这个发布者，成为cpp文件范围内的全局指针，方便在回调函数中，收一个发一个
    pubRobotSpeed = &pubspeed;
    pubScanPointer = &pubScan;
    ros::Rate rate(10);
    bool status = ros::ok();
    while (status)
    {
        ros::spinOnce();
        pcl::PointCloud<pcl::PointXYZ> pcl_points1;
        pcl::PointCloud<pcl::PointXYZ> pcl_points2;
        pcl::PointCloud<pcl::PointXYZ> pcl_registered;
        pcl::PointCloud<pcl::PointXYZI> pclI_registered;
        pcl::fromROSMsg(velodyne_points1, pcl_points1);
        // pcl::fromROSMsg(velodyne_points2, pcl_points2);

        for (int i = 0; i < pcl_points1.points.size(); i++)
        {
            pcl_registered.push_back(pcl_points1.points[i]);
        }
        // for (int i = 0; i < pcl_points2.points.size(); i++)
        // {
        //     pcl_registered.push_back(pcl_points2.points[i]);
        // }
        for (int i = 0; i < pcl_registered.points.size(); i++)
        {
            pcl::PointXYZI point;
            point.x = pcl_registered.points[i].x;
            point.y = pcl_registered.points[i].y;
            point.z = pcl_registered.points[i].z;
            point.intensity = 0.0;
            pclI_registered.push_back(point);
        }

        pcl::toROSMsg(pclI_registered, registered_points);
        registered_points.header.stamp = ros::Time::now();
        registered_points.header.frame_id = "map";
        pubScanPointer->publish(registered_points);

        tf::StampedTransform transform;
        double roll, pitch, yaw;
        try
        {
            // ros::Time(0)表示查询最接近当前时间的变换
            listener.lookupTransform("map", robotFrame, ros::Time(0), transform);
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("robotSimulator:%s", ex.what());
            ros::Duration(0.1).sleep();
            status = ros::ok();
            continue;
        }
        tf::Matrix3x3(transform.getRotation()).getEulerYPR(yaw, pitch, roll);
        geometry_msgs::Quaternion geoQuat = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
        // 启动后短时间内补偿里程计姿态（imu会由重力计识别）
        odomData.pose.pose.orientation = geoQuat;
        odomData.pose.pose.position.x = transform.getOrigin().getX();
        odomData.pose.pose.position.y = transform.getOrigin().getY();
        odomData.pose.pose.position.z = transform.getOrigin().getZ();
        odomData.header.frame_id = "map";
        odomData.child_frame_id = robotFrame;
        odomData.header.stamp = ros::Time::now();
        pubVehicleOdom.publish(odomData);
        status = ros::ok();
        rate.sleep();
    }
    return 0;
}

/*
==============================================================================================================================================================
==============================================================================================================================================================
   仿真下的函数表示
*/
// 点云数据回调函数
void scanHandler(const sensor_msgs::PointCloud2::ConstPtr &scanIn)
{
    // 原来的点云以velodyne为系，监听veloyne到map的tf转换，然后转换点云到那个系上。
    sensor_msgs::PointCloud2 scanData2;
    pcl_ros::transformPointCloud("map", *scanIn, scanData2, *tf_listener_ptr);
    velodyne_points1 = scanData2;
}
void scan2Handler(const sensor_msgs::PointCloud2::ConstPtr &scanIn)
{
    // 原来的点云以velodyne为系，监听veloyne到map的tf转换，然后转换点云到那个系上。
    sensor_msgs::PointCloud2 scanData2;
    pcl_ros::transformPointCloud("map", *scanIn, scanData2, *tf_listener_ptr);
    velodyne_points2 = scanData2;
}
