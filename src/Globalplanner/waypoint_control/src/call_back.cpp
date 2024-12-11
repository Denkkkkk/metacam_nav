#include "waypoint_control.h"
/**
 * @brief 路径规划状态回调
 *
 * @param status
 */
void waypoint_control::pathStatusCallback(const std_msgs::Bool::ConstPtr &status)
{
    path_status = *status;
    if (!path_status.data)
    {
        _midP_mode_ptr->is_midplanning = false;
        goal_path.poses.resize(0);
    }
    // 更新路径状态的时间
    pathState_update_time = ros::Time::now().toSec();
};

/**
 * @brief 全局路径回调
 *
 * @param point
 */
void waypoint_control::goalPathCallback(const nav_msgs::Path::ConstPtr &pathIn)
{
    int pathSize = pathIn->poses.size();
    goal_path.poses.resize(pathSize);
    for (int i = 0; i < pathSize; i++)
    {
        goal_path.poses[i].pose.position.x = pathIn->poses[i].pose.position.x;
        goal_path.poses[i].pose.position.y = pathIn->poses[i].pose.position.y;
        goal_path.poses[i].pose.position.z = pathIn->poses[i].pose.position.z;
    }
    pathNew = true;
};

/**
 * @brief 里程计回调
 *
 * @param odom
 */
void waypoint_control::odometryCallback(const nav_msgs::Odometry::ConstPtr &odom)
{
    double roll, pitch, yaw;
    geometry_msgs::Quaternion geoQuat = odom->pose.pose.orientation;
    tf::Matrix3x3(tf::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w)).getRPY(roll, pitch, yaw);
    vehicleYaw = yaw;
    vehicleX = odom->pose.pose.position.x;
    vehicleY = odom->pose.pose.position.y;
    vehicleZ = odom->pose.pose.position.z;
    vehicle_goal_dis = sqrt(pow(goal_point.pose.position.x - vehicleX, 2) + pow(goal_point.pose.position.y - vehicleY, 2));
}

/**
 * @brief 全局目标点回调
 *
 * @param point
 */
void waypoint_control::goalPointCallback(const geometry_msgs::PoseStamped::ConstPtr &point)
{
    goal_point = *point;
    goal_point.pose.position.x = point->pose.position.x;
    goal_point.pose.position.y = point->pose.position.y;
    double dis_goal = sqrt(pow(point->pose.position.x - goalX_last, 2) + pow(point->pose.position.y - goalY_last, 2));
    double dis_vehicle = sqrt(pow(point->pose.position.x - vehicleX, 2) + pow(point->pose.position.y - vehicleY, 2));
    // 防止新规划且没有全局路径会被上一个中间点阻拦
    if (!abs(point_last.pose.position.z - (-1)) < 0.01 && dis_goal > update_begin_dis)
    {
        _midP_mode_ptr->is_midplanning = false; // 清除中间点
    }
    if (dis_goal > update_begin_dis_goalPoint && dis_vehicle > update_begin_dis_vehicleDis)
    {
        // 重置恢复模式计时
        point_begin = ros::Time::now().toSec();
        // 清地图
        clearCostmap_client.call(srv);
        clear_justnow = true;
    }
    need_pub_goal = true;
    goalX_last = goal_point.pose.position.x;
    goalY_last = goal_point.pose.position.y;
};