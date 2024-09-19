#include "LpNode.h"

/**
 * @brief 外部关闭点云地图
 *
 * @param msg
 */
void LpNode::closeMapHandler(const std_msgs::Bool::ConstPtr &msg)
{
    if (lctlPtr->param.useMap && msg->data)
    {
        need_close_map = true;
    }
}

/**
 * @brief 全局目标点回调
 *
 * @param point
 */
void LpNode::goalHandler(const geometry_msgs::PoseStamped::ConstPtr &goal)
{
    // 目标点的x位置
    goalX = goal->pose.position.x;
    // 目标点的y位置
    goalY = goal->pose.position.y;
}

/**
 * @brief 速度值回调
 *
 * @param speed
 */
void LpNode::speedHandler(const std_msgs::Float32::ConstPtr &speed)
{
    // 计算速度与最大占比，用来限制路径规模
    joySpeed = speed->data;
    if (joySpeed > maxSpeed)
    {
        joySpeed = maxSpeed;
    }
}

/**
 * @brief 是否开启障碍物检测回调
 *
 * @param checkObs
 */
void LpNode::checkObstacleHandler(const std_msgs::Bool::ConstPtr &checkObs)
{
    lctlPtr->param.checkObstacle = checkObs->data; // 将获取的布尔值数据赋值给名为 checkObstacle 的全局变量。这将用于控制障碍物检测的开启位
}

/**
 * @brief 里程计回调
 *
 * @param odom
 */
void LpNode::odometryHandler(const nav_msgs::Odometry::ConstPtr &odom)
{
    odomTime = odom->header.stamp.toSec();

    double roll, pitch, yaw;
    geometry_msgs::Quaternion geoQuat = odom->pose.pose.orientation;
    tf::Matrix3x3(tf::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w)).getRPY(roll, pitch, yaw);
    vehicleRoll = roll;
    vehiclePitch = pitch;
    vehicleYaw = yaw;
    vehicleX = odom->pose.pose.position.x;
    vehicleY = odom->pose.pose.position.y;
    vehicleZ = odom->pose.pose.position.z;

    // 特定区域缩小到点规划范围
    // if (goalClearCenter_x - 1 < vehicleX && vehicleX < goalClearCenter_x + 1 && goalClearCenter_y - 1 < vehicleY && vehicleY < goalClearCenter_y + 1)
    // {
    //     marker.scale.x = 2;
    //     marker.scale.y = 2;
    //     marker.header.stamp = ros::Time::now();
    //     pubMarker.publish(marker);
    //     goalClearRange = 0;
    // }
    // else
    // {
    //     goalClearRange = defaultGoalDis;
    // }
}

/**
 * @brief 地形点云回调
 *
 * @param terrainCloud2
 */
void LpNode::terrainCloudHandler(const sensor_msgs::PointCloud2::ConstPtr &terrainCloud2)
{
    // 首先将上一帧的点云清除
    terrainCloud->clear();
    // 将本帧的点云转化为ROS格式
    pcl::fromROSMsg(*terrainCloud2, *terrainCloud);
    pcl::PointXYZI point;
    // terrainCloudCrop为经过地形裁剪之后的点云
    terrainCloudCrop->clear();
    if (lctlPtr->param.useMap)
    {
        *terrainCloudCrop = *terrainMapRecord_pcl;
    }
    else
    {
        *terrainCloudCrop = *terrainMapRecord_pcl_protect;
    }
    int terrainCloudSize = terrainCloud->points.size();
    // 排除掉->太远，（太低或代价评价）
    for (int i = 0; i < terrainCloudSize; i++)
    {
        point = terrainCloud->points[i];
        float pointX = point.x;
        float pointY = point.y;
        // 计算点云到车体的距离
        float dis = sqrt((pointX - vehicleX) * (pointX - vehicleX) + (pointY - vehicleY) * (pointY - vehicleY));
        // 在CMU团队自己设计的地面分割(地形分析)，intensity定义为障碍物点云距离地面点云（最低点云位置）的相对高度
        // 只保存满足下列条件的点云
        // ①距离车体小于adjacentRange的点云
        // ②使用对于低于阈值的点云进行路径评分useCost，只要障碍物点云距离地面点云的相对高度大于obstacleHeightThre（地形裁剪）
        if (dis < lctlPtr->param.adjacentRange && (point.intensity > lctlPtr->param.obstacleHeightThre || lctlPtr->param.useCost))
        {
            terrainCloudCrop->push_back(point);
        }
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr terrainAddPoints(new pcl::PointCloud<pcl::PointXYZI>);
    terrainAddPoints->clear();
    // 遍历点云插值
    if (lctlPtr->param.add_point_radius > 0.01)
    {
        // 先做一次降采样
        terrainCloudDwz->clear();
        terrainDwzFilter.setInputCloud(terrainCloudCrop);
        terrainDwzFilter.filter(*terrainCloudDwz);
        *terrainCloudCrop = *terrainCloudDwz;
        terrainCloudDwz->clear();
        for (int i = 0; i < terrainCloudCrop->points.size(); i++)
        {
            terrainCloudDwz->push_back(terrainCloudCrop->points[i]);
            // 60度插值一次
            for (int j = 1; j < 6; j++)
            {
                point.x = terrainCloudCrop->points[i].x + lctlPtr->param.add_point_radius * cos(j * 60 * M_PI / 180);
                point.y = terrainCloudCrop->points[i].y + lctlPtr->param.add_point_radius * sin(j * 60 * M_PI / 180);
                point.z = terrainCloudCrop->points[i].z;
                terrainCloudDwz->push_back(point);
                terrainAddPoints->push_back(point);
            }
        }
        *terrainCloudCrop = *terrainCloudDwz;
    }
    // 发布插值点云
    sensor_msgs::PointCloud2 terrainAddPoints2;
    pcl::toROSMsg(*terrainAddPoints, terrainAddPoints2);
    terrainAddPoints2.header.stamp = ros::Time::now();
    terrainAddPoints2.header.frame_id = "map";
    pubAddPoints.publish(terrainAddPoints2);
    terrainAddPoints->clear();

    //  最后做一次降采样
    terrainCloudDwz->clear();
    terrainDwzFilter.setInputCloud(terrainCloudCrop);
    terrainDwzFilter.filter(*terrainCloudDwz);
    newTerrainCloud = true;
    plannerCloud->clear();
    *plannerCloud = *terrainCloudDwz;
}

/**
 * @brief 补充点云(2d雷达)回调
 *
 * @param laserCloud2
 */
void LpNode::addCloudHandler(const sensor_msgs::PointCloud2ConstPtr &addPoints)
{
    addedObstacles->clear();
    addedObstaclesCrop->clear();
    pcl::fromROSMsg(*addPoints, *addedObstacles);
    pcl::PointXYZI point;
    for (long unsigned int i = 0; i < addedObstacles->points.size(); i++)
    {
        point = addedObstacles->points[i];
        float pointX = point.x;
        float pointY = point.y;
        // 计算点云到车体的距离
        float dis = sqrt((pointX - vehicleX) * (pointX - vehicleX) + (pointY - vehicleY) * (pointY - vehicleY));
        if (dis < lctlPtr->param.adjacentRange && (point.intensity > lctlPtr->param.obstacleHeightThre || lctlPtr->param.useCost))
        {
            addedObstaclesCrop->push_back(point);
        }
    }
    get_addedObstacles = true;
}

void LpNode::globalPointHandler(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    goal_point = *msg;
}