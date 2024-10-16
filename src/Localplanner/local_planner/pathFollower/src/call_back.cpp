#include "pathFollower.h"

/**
 * @brief 里程计回调
 *
 * @param odomIn
 */
void RoboCtrl::odomHandler(const nav_msgs::Odometry::ConstPtr &odomIn)
{
    odomTime = odomIn->header.stamp.toSec();
    double roll, pitch, yaw;
    geometry_msgs::Quaternion geoQuat = odomIn->pose.pose.orientation;
    tf::Matrix3x3(tf::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w)).getRPY(roll, pitch, yaw);
    vehicleRoll = roll;
    vehiclePitch = pitch;
    vehicleYaw = yaw;

    // 传感器坐标系转到车体中心坐标系
    vehicleX = odomIn->pose.pose.position.x;
    vehicleY = odomIn->pose.pose.position.y;
    vehicleZ = odomIn->pose.pose.position.z;

    odom_update_time = ros::Time::now().toSec();
}

/**
 * @brief 局部目标点回调
 *
 * @param goal
 */
void RoboCtrl::goalHandler(const geometry_msgs::PoseStamped::ConstPtr &goal)
{
    // 目标点的x位置
    way_point = *goal;
    use_real_goal = true;
}

/**
 * @brief 订阅经过localPlanner处理过后的最佳路径path
 * @param pathIn
 */
void RoboCtrl::pathHandler(const nav_msgs::Path::ConstPtr &pathIn)
{
    int pathSize = pathIn->poses.size();
    path.poses.resize(pathSize);
    // 转到虚拟车头
    if (pctlPtr->param.use_virtual_head)
    {
        vehicleYawRec = virture_headDir;
        for (int i = 0; i < pathSize; i++)
        {
            double pointX = pathIn->poses[i].pose.position.x;
            double pointY = pathIn->poses[i].pose.position.y;
            double v_to_vh = virture_headDir - vehicleYaw;

            path.poses[i].pose.position.x = pointX * cos(v_to_vh) + pointY * sin(v_to_vh);
            path.poses[i].pose.position.y = -pointX * sin(v_to_vh) + pointY * cos(v_to_vh);
        }
    }
    else
    {
        vehicleYawRec = vehicleYaw;
        for (int i = 0; i < pathSize; i++)
        {
            path.poses[i].pose.position.x = pathIn->poses[i].pose.position.x;
            path.poses[i].pose.position.y = pathIn->poses[i].pose.position.y;
            path.poses[i].pose.position.z = pathIn->poses[i].pose.position.z;
        }
    }

    // 记录获取到路径信息瞬间的车体位置和姿态
    vehicleXRec = vehicleX;
    vehicleYRec = vehicleY;
    vehicleZRec = vehicleZ;
    vehicleRollRec = vehicleRoll;
    vehiclePitchRec = vehiclePitch;
    pathPointID = 0;
    pathInit = true;
}

/**
 * @brief 全局路径回调
 *
 * @param speed
 */
void RoboCtrl::goalPathCallback(const nav_msgs::Path::ConstPtr &pathIn)
{
    nav_msgs::Path goal_path_temp;
    int pathSize = pathIn->poses.size();
    goal_path_temp.poses.resize(pathSize);
    if (pctlPtr->param.use_virtual_head)
    {
        vehicleYawRec = virture_headDir;
        for (int i = 0; i < pathSize; i++)
        {
            double pointX = pathIn->poses[i].pose.position.x - vehicleX;
            double pointY = pathIn->poses[i].pose.position.y - vehicleY;
            // 将全局路径转换到车体坐标系
            goal_path_temp.poses[i].pose.position.x = pointX * cos(virture_headDir) + pointY * sin(virture_headDir);
            goal_path_temp.poses[i].pose.position.y = -pointX * sin(virture_headDir) + pointY * cos(virture_headDir);
            double dis = sqrt(pow(goal_path_temp.poses[i].pose.position.x, 2) + pow(goal_path_temp.poses[i].pose.position.y, 2));
            if (dis > 2.0 || i == pathSize - 1)
            {
                goal_path.poses.resize(i + 1);
                for (int j = 0; j < i + 1; j++)
                {
                    goal_path.poses[j] = goal_path_temp.poses[j];
                }
                break;
            }
        }
    }
    else
    {
        vehicleYawRec = vehicleYaw;
        for (int i = 0; i < pathSize; i++)
        {
            double pointX = pathIn->poses[i].pose.position.x - vehicleX;
            double pointY = pathIn->poses[i].pose.position.y - vehicleY;
            // 将全局路径转换到车体坐标系
            goal_path_temp.poses[i].pose.position.x = pointX * cos(vehicleYaw) + pointY * sin(vehicleYaw);
            goal_path_temp.poses[i].pose.position.y = -pointX * sin(vehicleYaw) + pointY * cos(vehicleYaw);
            double dis = sqrt(pow(goal_path_temp.poses[i].pose.position.x, 2) + pow(goal_path_temp.poses[i].pose.position.y, 2));
            if (dis > 2.0 || i == pathSize - 1)
            {
                goal_path.poses.resize(i + 1);
                for (int j = 0; j < i + 1; j++)
                {
                    goal_path.poses[j] = goal_path_temp.poses[j];
                }
                break;
            }
        }
    }
    // 记录获取到路径信息瞬间的车体位置和姿态
    vehicleXRec = vehicleX;
    vehicleYRec = vehicleY;
    vehicleZRec = vehicleZ;
    vehicleRollRec = vehicleRoll;
    vehiclePitchRec = vehiclePitch;

    pathPointID = 0;
    pathInit = true;
};

/**
 * @brief 停止模式回调
 *
 * @param speed
 */
void RoboCtrl::stopHandler(const std_msgs::Bool::ConstPtr &stop)
{
    pctlPtr->param.safetyStop = stop->data;
}

/**
 * @brief 全局目标点回调
 *
 * @param point
 */
void RoboCtrl::goalPointCallback(const geometry_msgs::PoseStamped::ConstPtr &point)
{
    goal_point_origin = *point;
    // use_real_goal = true;
};

/**
 * @brief 全局路径规划状态回调
 *
 * @param status
 */
void RoboCtrl::pathStatusCallback(const std_msgs::Bool::ConstPtr &status)
{
    path_status = *status;
};

/**
 * @brief 规划模式回调
 *
 * @param msg
 */
void RoboCtrl::controlModeCallback(const std_msgs::Int8::ConstPtr &msg)
{
    switch (msg->data)
    {
    case 0:
        control_mode = GOALDIRECT;
        break;
    case 1:
        control_mode = MIDPlanning;
        break;
    case 2:
        control_mode = GUIDEPlanning;
        break;
    case 3:
        control_mode = TRANSVERSE;
        break;
    case 4:
        control_mode = SWING;
        break;
    }
}

/**
 * @brief 点云方向限制速度的回调
 *
 * @param slowDown
 */
void RoboCtrl::slowDownHandler(const std_msgs::Float32::ConstPtr &slowDown)
{
    slowDown1 = slowDown->data;
    slowDown1_update_time = ros::Time::now().toSec();
}