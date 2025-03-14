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
 * @brief 局部目标点回调函数
 * @param goal
 * @note 计算记录的way_point和接收到的way_point的距离，如果距离大于0.1m，则更新way_point且use_real_goal为true
 */
void RoboCtrl::goalHandler(const geometry_msgs::PoseStamped::ConstPtr &goal)
{
    // 目标点的x位置
    double distance = sqrt(pow(way_point.pose.position.x - goal->pose.position.x, 2) + pow(way_point.pose.position.y - goal->pose.position.y, 2));
    if (distance > 0.1)
    {
        way_point = *goal;
        use_real_goal = true;
    }
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
    if (pctlPtr->get_params().use_virtual_head)
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
    if (pctlPtr->get_params().use_virtual_head)
    {
        vehicleYawRec = virture_headDir;
        for (int i = 0; i < pathSize; i++)
        {
            if (i == 0 || i == 1)
            {
                continue;
            }
            double pointX = pathIn->poses[i].pose.position.x - vehicleX;
            double pointY = pathIn->poses[i].pose.position.y - vehicleY;
            // 将全局路径转换到车体坐标系
            goal_path_temp.poses[i].pose.position.x = pointX * cos(virture_headDir) + pointY * sin(virture_headDir);
            goal_path_temp.poses[i].pose.position.y = -pointX * sin(virture_headDir) + pointY * cos(virture_headDir);
            double dis = sqrt(pow(goal_path_temp.poses[i].pose.position.x, 2) + pow(goal_path_temp.poses[i].pose.position.y, 2));
            if (dis > 2.0 || i == pathSize - 1)
            {
                goal_path.poses.resize(i + 1 - 2);
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

/**比
 * @brief 停止模式回调
 *
 * @param speed
 */
void RoboCtrl::stopHandler(const std_msgs::Bool::ConstPtr &stop)
{
    safetyStop = stop->data;
}

/**
 * @brief 全局目标点回调
 *
 * @param point
 */
void RoboCtrl::goalPointCallback(const geometry_msgs::PoseStamped::ConstPtr &point)
{
    goal_point_origin = *point;
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
        control_mode = RECOVER;
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
    local_slowDown = slowDown->data;
    local_slowDown_update_time = ros::Time::now().toSec();
}

void RoboCtrl::terrainCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &terrainCloud2)
{
    pcl::PointCloud<pcl::PointXYZI> terrainCloud;
    // 将本帧的点云转化为ROS格式
    pcl::fromROSMsg(*terrainCloud2, terrainCloud);
    pcl::PointXYZI point;
    terrainCloud_minDis = std::numeric_limits<float>::max(); // 初始化为类型最大值

    // 排除掉->太远，（太低或代价评价）
    for (int i = 0; i < terrainCloud.points.size(); i++)
    {
        point = terrainCloud.points[i];
        float pointX = point.x;
        float pointY = point.y;
        // 计算点云到车体的距离
        float dis = sqrt((pointX - vehicleX) * (pointX - vehicleX) + (pointY - vehicleY) * (pointY - vehicleY));
        // 使用对于低于阈值的点云进行路径评分useCost，只要障碍物点云距离地面点云的相对高度大于obstacleHeightThre（地形裁剪）
        if (point.intensity > pctlPtr->get_params().obstacleHeightThre)
        {
            if (dis < terrainCloud_minDis)
            {
                terrainCloud_minDis = dis;
            }
        }
    }
}