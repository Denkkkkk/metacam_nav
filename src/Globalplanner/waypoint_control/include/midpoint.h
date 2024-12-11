/**
 * @file midpoint.h
 * @author 李东权 1327165187@qq.com
 * @brief 中间点规划模式
 * @version 1.0
 * @date 2024-07-22
 *
 * @copyright Copyright (c) 2024
 *
 */
#pragma once
#include "extern.h"
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>

class MidPointMode
{
public:
    MidPointMode();
    void load_param();
    void countMidPoint(bool &pathNew);
    void pub_turn_point();

    geometry_msgs::PoseStamped mid_point;
    // 转弯点规划外部参数
    bool is_midplanning = false;
    bool mid_point_exist = false;
    double get_midPlanner_dis;
    double need_midplan_dis = 2.0;
    // 引导点外部参数
    bool pub_guide = false;
    bool use_midplan = true;
    bool use_guide = true;
    double guide_dis = 1.0;

private:
    ros::NodeHandle _nh;
    ros::NodeHandle _nhPrivate = ros::NodeHandle("~");
    ros::Publisher _pubTurnPoint;
    /**
     * @brief 转弯点条件参数
     *
     */
    double _lenth_forward;
    double _lenth_back;
    double _curvature_threshold;
    double _no_midPlanner_forward;
    double _mid_point_back;
    double _update_begin_dis;
    /**
     * @brief 引导点条件参数
     *
     */
    bool _guide_exist = false;
};