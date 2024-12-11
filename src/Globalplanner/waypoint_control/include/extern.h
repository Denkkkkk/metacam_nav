#pragma once
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <nav_msgs/Path.h>
#include <ros/ros.h>

extern double vehicleX, vehicleY, vehicleZ, vehicleYaw;

extern geometry_msgs::PoseStamped point_last; // 上一个给点
extern nav_msgs::Path goal_path;
