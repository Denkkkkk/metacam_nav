#pragma once

#include <chrono>
#include <math.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int8.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>

#include "pathFollower/parameters.h"
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/JointState.h>

enum ControlMode
{
    GOALDIRECT,
    MIDPlanning,
    GUIDEPlanning,
    TRANSVERSE,
    SWING
};

class RoboCtrl
{
public:
    ParamControl *pctlPtr;
    int pub_rate;
    RoboCtrl();
    ~RoboCtrl() { delete pctlPtr; }
    void pure_persuit();
    void pubVehicleSpeed(const double vehicleSpeed);
    void pubVehicleSpeed_goalDir(const double vehicleSpeed, const double goal_dir);
    void slowStop();

private:
    void odomHandler(const nav_msgs::Odometry::ConstPtr &odomIn);
    void pathHandler(const nav_msgs::Path::ConstPtr &pathIn);
    void stopHandler(const std_msgs::Bool::ConstPtr &stop);
    void goalHandler(const geometry_msgs::PoseStamped::ConstPtr &goal);
    void slowDownHandler(const std_msgs::Float32::ConstPtr &slowDown);
    void goalPointCallback(const geometry_msgs::PoseStamped::ConstPtr &point);
    void pathStatusCallback(const std_msgs::Bool::ConstPtr &status);
    void goalPathCallback(const nav_msgs::Path::ConstPtr &pathIn);
    void controlModeCallback(const std_msgs::Int8::ConstPtr &msg);
    void imuCallback(const sensor_msgs::Imu::ConstPtr &imu);
    void slowCallback(const std_msgs::Float32::ConstPtr &slowDown);
    void slowDown();
    void terrainCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &terrainCloud2);

    ros::NodeHandle nh;
    ros::NodeHandle nhPrivate = ros::NodeHandle("~");
    ros::Subscriber subOdom;
    ros::Subscriber subPath;
    ros::Subscriber subSpeed;
    ros::Subscriber subStop;
    ros::Subscriber subGoal;
    ros::Subscriber subSlowDown;
    ros::Subscriber subSerial;
    ros::Subscriber subGlobalPoint;
    ros::Subscriber subPathStatus;
    ros::Subscriber subGoalPath;
    ros::Subscriber subControlMode;
    ros::Subscriber subTerrainCloud;
    ros::Publisher pubSpeed;
    ros::Publisher pubCmd_vel;
    ros::Publisher pubGoalPathDir;
    ros::Publisher pubVirHeadDir;
    ros::Publisher pubGetGoal;

    const double PI = 3.1415926;

    float vehicleX = 0;
    float vehicleY = 0;
    float vehicleZ = 0;
    float vehicleRoll = 0;
    float vehiclePitch = 0;
    float vehicleYaw = 0;
    float vehicleXRec = 0;
    float vehicleYRec = 0;
    float vehicleZRec = 0;
    float vehicleRollRec = 0;
    float vehiclePitchRec = 0;
    float vehicleYawRec = 0;
    float vehicleYawRate = 0;
    float vehicleSpeed = 0;

    double odomTime = 0;
    int pathPointID = 0;
    bool pathInit = false;
    bool navFwd = true;
    double switchTime = 0;
    double maxSpeed1 = 0;
    float cloudToSpeedK = 0;
    float cloudToSpeedB = 0;
    double switchTimeThre = 1.5; // 允许1.5s翻转一次车头方向
    std::string cmdTopic;
    geometry_msgs::PoseStamped goal_point;
    geometry_msgs::PoseStamped way_point;
    geometry_msgs::PoseStamped goal_point_origin;
    std_msgs::Bool path_status;
    nav_msgs::Path goal_path;
    geometry_msgs::PoseStamped goal_path_dir;
    float endPathDis_now;
    float endGoalDis_now;
    float virture_endGoalDis_now;
    float vehicleXRel;
    float vehicleYRel;
    double virture_headDir = 0;
    double goalSlow_K;
    double pathSlow_K;
    ControlMode control_mode = GOALDIRECT;
    double virture_goalX;
    double virture_goalY;
    bool use_real_goal = true; // 是否使用真实目标点，虚拟目标点为到点后的当前点
    nav_msgs::Path path;
    std_msgs::Bool get_goal;
    double transTime_slow_begin;
    std_msgs::Float32 car_speed;
    geometry_msgs::Twist cmd_vel;
    double local_slowDown;
    double local_slowDown_update_time;
    int mid_slow_delay = 15;
    std::string ns;
    std::string robot_frame;
    double odom_update_time = 0;
    bool use_two_forward = false;
    bool safetyStop = false;
    float terrainCloud_minDis = 100;
    bool near_cloud_stop = false;
};