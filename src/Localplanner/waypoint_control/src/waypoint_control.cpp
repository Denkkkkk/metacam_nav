/**
 * @file waypoint_control.cpp
 * @author 李东权 (1327165187@qq.com)
 * @brief  局部规划器的控制系统
 * @version 1.0
 * @date 2024-04-21
 *
 * @copyright Copyright (c) 2024
 *
 */
#include <waypoint_control.h>

waypoint_control::waypoint_control()
{
    subGoalPoint = nh.subscribe("/move_base_simple/goal", 5, &waypoint_control::goalPointCallback, this);
    subGoalPath = nh.subscribe("/move_base/GlobalPlanner/plan", 5, &waypoint_control::goalPathCallback, this);
    subPathStatus = nh.subscribe("/move_base/GlobalPlanner/goal_plan_status", 1, &waypoint_control::pathStatusCallback, this);
    subOdometry = nh.subscribe("/odom_interface", 5, &waypoint_control::odometryCallback, this);

    clearCostmap_client = nh.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");
    pubWayPoint = nh.advertise<geometry_msgs::PoseStamped>("/way_point", 5);
    pubControlMode = nh.advertise<std_msgs::Int8>("/control_mode", 1);
    pubCloseMap = nh.advertise<std_msgs::Bool>("/close_map", 2);

    nhPrivate.param("use_recovery_mode", use_recovery_mode, false);
    nhPrivate.param("lateral_dis", lateral_dis, 0.8);
    nhPrivate.param("lateral_goal_dis", lateral_goal_dis, 0.8);
    nhPrivate.param("lateral_time", lateral_time, 2.0);
    nhPrivate.param("update_begin_dis", update_begin_dis, 0.1);
    nhPrivate.param("default_point_time", default_point_time, 5.0);
    nhPrivate.param("reach_goal_dis", reach_goal_dis, 0.5);
    nhPrivate.param("goal_point_only", goal_point_only, false);
    nhPrivate.param("update_begin_dis_goalPoint", update_begin_dis_goalPoint, 4.0);
    nhPrivate.param("update_begin_dis_vehicleDis", update_begin_dis_vehicleDis, 5.0);

    _midP_mode_ptr = std::unique_ptr<MidPointMode>(new MidPointMode());
    path_status.data = false;
    need_pub_goal = false;
    goal_path.poses.resize(0);
    point_last.pose.position.x = 0;
    point_last.pose.position.y = 0;
    point_last.pose.position.z = -1;
    goal_point.pose.position.x = 0;
    goal_point.pose.position.y = 0;
}

// 发布 WayPoint 到话题 /way_point
void waypoint_control::publishWayPoint(geometry_msgs::PoseStamped &point)
{
    point.header.frame_id = "map";
    point.header.stamp = ros::Time::now();
    double dis = sqrt(pow(point.pose.position.x - point_last.pose.position.x, 2) + pow(point.pose.position.y - point_last.pose.position.y, 2));
    if (dis >= update_begin_dis || !_midP_mode_ptr->use_guide || (abs(point_last.pose.position.z - (-1)) < 0.01))
    {
        point_begin = ros::Time::now().toSec();
        ROS_INFO("update_goal_point,point_begin:%f", point_begin);
    }
    pubWayPoint.publish(point);
    point_last = point;
}

// 更新中间点持续时间
void waypoint_control::updateDuration()
{
    if (vehicle_goal_dis < reach_goal_dis || (abs(point_begin - (-1)) < 0.01))
    {
        point_duration = -1;
        return;
    }
    else
    {
        point_end = ros::Time::now().toSec();
        if (debug)
        {
            ROS_INFO("point_end:%f", point_end);
        }
        point_duration = point_end - point_begin;
    }
}

void waypoint_control::cancelLateraling()
{
    /**
     * @brief 取消横移标志位
     * 1. 正在横移
     * 2. 横移开始时间到现在时间大于横移持续时间
     */
    if (is_lateraling == true)
    {
        lateral_end = ros::Time::now().toSec();
        if (debug)
        {
            ROS_INFO("lateral_end:%f", lateral_end);
        }
        lateral_duration = lateral_end - lateral_begin;
        if (lateral_duration > lateral_time)
        {
            is_lateraling = false;
            // 刚做完横移，没有新mid_point就不能再进入横移了
            point_begin = -1;
            if (debug)
            {
                ROS_INFO("point_begin:%f", point_begin);
            }
        }
    }
    // 更新waypoint持续时间必须在横移结束判断之后
    updateDuration();
}

void waypoint_control::updateCase()
{
    /**
     * @brief 更新路径状态的接收时间
     *
     */
    if (ros::Time::now().toSec() - pathState_update_time > 1.5)
    {
        path_status.data = false;
    }
    /**
     * @brief 转弯点较远或者到达，清除转弯点
     *
     */
    if (_midP_mode_ptr->is_midplanning)
    {
        double midpoint_veh_dis = sqrt(pow(point_last.pose.position.x - vehicleX, 2) + pow(point_last.pose.position.y - vehicleY, 2));
        if (midpoint_veh_dis < _midP_mode_ptr->get_midPlanner_dis || midpoint_veh_dis > _midP_mode_ptr->guide_dis + 30)
        {
            _midP_mode_ptr->is_midplanning = false;
        }
    }

    /**
     * @brief 到全局目标点，规划结束
     *
     */
    vehicle_goal_dis = sqrt(pow(goal_point.pose.position.x - vehicleX, 2) + pow(goal_point.pose.position.y - vehicleY, 2));
    if (vehicle_goal_dis < reach_goal_dis)
    {
        // 到点后，清除横移标志位，清除全局路径
        is_lateraling = false;
        _midP_mode_ptr->is_midplanning = false;
        goal_path.poses.resize(0);
        control_mode = GOALDIRECT;
        point_begin = ros::Time::now().toSec(); // 重置时间
        return;
    }

    /**
     * @brief 进入横移模式尝试跳出局部最优
     * 1. 与全局点距离大于lateral_goal_dis
     * 2. 到当前waypoint超时
     * 3. 正在上个横移进行中is_lateraling = true
     * 4. 全局路径丢失超时
     */
    cancelLateraling(); // 判断横移结束并更新waypoint持续时间
    if ((use_recovery_mode &&vehicle_goal_dis > lateral_goal_dis && point_duration > default_point_time) || is_lateraling)
    {
        control_mode = RECOVER;
        return;
    }
    /**
     * @brief 使用中间点控制
     * 1.当全局点与车辆距离大于一定值
     * 2.没有找到大曲率点
     * 3.全局路径规划失败
     */
    if (vehicle_goal_dis > _midP_mode_ptr->need_midplan_dis && path_status.data == true && (_midP_mode_ptr->use_midplan || _midP_mode_ptr->use_guide))
    {
        _midP_mode_ptr->countMidPoint(pathNew);
        if (_midP_mode_ptr->mid_point_exist || _midP_mode_ptr->is_midplanning)
        {
            control_mode = MIDPOINT;
            return;
        }
    }
    /**
     * @brief 直接使用goal_point控制
     * 1.优先级在最后
     */
    control_mode = GOALDIRECT;
    return;
}

// --------------------------------------
// 控制状态流转
// 主要发布消息：
// 1. control_mode: 发布到话题 /control_mode，消息类型 std_msgs::Int8
// 2. way_point: 发布到话题 /way_point，消息类型 geometry_msgs::PoseStamped
// --------------------------------------
void waypoint_control::run()
{
    if (goal_point_only)    // 仅关注目标点
    {
        control_mode = GOALDIRECT;
    }
    else
    {
        if (clear_justnow)
        {
            // ros::Duration(0.06).sleep();
            clear_justnow = false;
        }
        if (vehicle_goal_dis < lateral_goal_dis) // 恢复模式距离内不累计时间
        {
            point_begin = ros::Time::now().toSec();
        }
        updateCase();
    }
    // ----------------------------------
    // 状态流转
    // ----------------------------------
    std::string control_mode_str;   // 控制状态字符串
    switch (control_mode)
    {
        // ----------------恢复状态------------------
    case RECOVER:
        control_mode_str = "RECOVER";
        control_mode_pub = pubRECOVER;
        if (!is_lateraling) // 如果不是横移中
        {
            lateral_begin = ros::Time::now().toSec();
            if (debug)
            {
                ROS_INFO("lateral_begin:%f", lateral_begin);
            }
            point_begin = ros::Time::now().toSec();
            if (debug)
            {
                ROS_INFO("into_lateral_point_begin:%f", point_begin);
            }
            is_lateraling = true;
            need_pub_goal = true;                   // 做完横移后要使能回来全局目标点
            _midP_mode_ptr->is_midplanning = false; // 清除中间点
            point_last.pose.position.z = -1;        // 重置新点时间累计
            clearCostmap_client.call(srv);          // 清除代价地图
            clear_justnow = true;
            std_msgs::Bool close_map;
            close_map.data = true;
            pubCloseMap.publish(close_map); // 完成保护地图构建后，恢复模式清除局部地图

            // 计算当前车体到全局目标点的射线方向,发布一个反方向0.5m的waypoint
            // double angle = atan2(goal_point.pose.position.y - vehicleY, goal_point.pose.position.x - vehicleX);
            // geometry_msgs::PoseStamped lateral_point;
            // lateral_point.header.frame_id = "map";
            // lateral_point.pose.position.x = vehicleX - lateral_dis * cos(angle);
            // lateral_point.pose.position.y = vehicleY - lateral_dis * sin(angle);
            // lateral_point.pose.orientation.z = 0;
            // lateral_point.pose.orientation.w = 1;
            // lateral_point.header.stamp = ros::Time::now();
            // publishWayPoint(lateral_point);
        }
        break;
        // ----------------中间点状态------------------
    case MIDPOINT:
        _midP_mode_ptr->pub_turn_point();
        if (_midP_mode_ptr->is_midplanning) // 中间点规划中
        {
            control_mode_str = "MIDPlanning";
            control_mode_pub = pubMIDPlanning;
        }
        else    // 引导线规划中
        {
            control_mode_str = "GUIDEPlanning";
            control_mode_pub = pubGUIDEPlanning;
        }
        if (pathNew || (_midP_mode_ptr->pub_guide))
        {
            publishWayPoint(_midP_mode_ptr->mid_point);
            pathNew = false;
            need_pub_goal = true;
        }
        break;
        // ----------------目标点状态------------------
    case GOALDIRECT:
        control_mode_str = "GOALDIRECT";
        control_mode_pub = pubGOALDIRECT;
        if (need_pub_goal)
        {
            publishWayPoint(goal_point);
            need_pub_goal = false;
        }
        break;
    }
    if (control_mode != RECOVER)
        ROS_WARN("control_mode: %s", control_mode_str.c_str());
    else
        ROS_ERROR("control_mode: %s", control_mode_str.c_str());
    std_msgs::Int8 control_mode_temp;
    control_mode_temp.data = control_mode_pub;
    pubControlMode.publish(control_mode_temp);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "waypoint_control");
    waypoint_control wpControl;
    ros::Rate rate(50);
    bool status = ros::ok();
    while (status)
    {
        ros::spinOnce();
        wpControl.run();
        status = ros::ok();
        rate.sleep();
    }
    return 0;
}