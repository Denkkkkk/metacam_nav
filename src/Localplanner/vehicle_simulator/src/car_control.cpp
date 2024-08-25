/**
 * @file car_control.cpp
 * @author 李东权 (1327165187@qq.com)
 * @brief 读取速度信息，控制Gazebo仿真环境中模型或实车的状态。
 * @version 1.0
 * @date 2024-1.18
 *
 * @copyright Copyright (c) 2024
 *
 */
#include "msg_process/send_data.h"
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <ros/ros.h>
#include <thread>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Bool.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

using namespace std;

const double PI = 3.1415926;
float vehicleX = 0;
float vehicleY = 0;
float vehicleZ = 0;
float vehicleRoll = 0;
float vehiclePitch = 0;
float vehicleYaw = 0;
float vehicleYawRate = 0;
float vehicleSpeedX = 0;
float vehicleSpeedY = 0;

msg_process::send_data serial_send_datas; // 发送给串口的信息

ros::Publisher serial_pub;
ros::Publisher corner_point_pub;
ros::Subscriber subSpeed;
ros::Subscriber subDecision;
ros::Subscriber subOdom;
ros::Subscriber subSlowSpin;
ros::Subscriber subPathStatus;
ros::Subscriber subGoalPath;

bool is_key_control = false;
bool slow_base_spin_flag = false;
bool path_status = false;
uint8_t decision_base_spin = 2U;
nav_msgs::Path goal_path;
double path_last_x, path_last_y;

void speedHandler(const geometry_msgs::Twist::ConstPtr &speedIn); // 速度回调
void odomHandler(const nav_msgs::Odometry::ConstPtr &odom);       // 里程计回调

int main(int argc, char **argv)
{
    ros::init(argc, argv, "car_control");
    ros::NodeHandle nh;
    ros::NodeHandle nhPrivate = ros::NodeHandle("~");
    nhPrivate.getParam("is_key_control", is_key_control);

    serial_pub = nh.advertise<msg_process::send_data>("/serial_send_data", 10);
    corner_point_pub = nh.advertise<geometry_msgs::PointStamped>("/corner_point", 10);

    // 键盘和导航控制切换
    if (!is_key_control)
    {
        subSpeed = nh.subscribe<geometry_msgs::Twist>("/cmd_vel", 1, speedHandler);
    }
    subOdom = nh.subscribe<nav_msgs::Odometry>("/Odometry", 1, odomHandler);

    // 在新线程中以一定频率向串口发布数据
    ros::Rate rate(90);
    thread send_serial_thread(
        [&rate]() -> void {
            while (ros::ok())
            {
                serial_pub.publish(serial_send_datas);
                rate.sleep();
            }
        });
    // 在主线程中执行回调实时更新数据
    ros::spin();
    return 0;
}

void speedHandler(const geometry_msgs::Twist::ConstPtr &speedIn)
{
    serial_send_datas.vx = speedIn->linear.x;
    // serial_send_datas.vy = speedIn->linear.y;
    serial_send_datas.w = speedIn->angular.z;
}

void odomHandler(const nav_msgs::Odometry::ConstPtr &odom)
{
    double roll, pitch, yaw;
    geometry_msgs::Quaternion geoQuat = odom->pose.pose.orientation;
    tf::Matrix3x3(tf::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w)).getRPY(roll, pitch, yaw);
    // 车体的roll，pitch，yaw角
    vehicleRoll = roll;
    vehiclePitch = pitch;
    vehicleYaw = yaw;
    vehicleX = odom->pose.pose.position.x;
    vehicleY = odom->pose.pose.position.y;
    vehicleZ = odom->pose.pose.position.z;
}
