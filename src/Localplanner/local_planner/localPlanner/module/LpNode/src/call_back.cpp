#include "LpNode.h"

/**
 * @brief 外部关闭点云地图
 *
 * @param msg
 */
void LpNode::closeMapHandler(const std_msgs::Bool::ConstPtr &msg)
{
    if (lctlPtr->get_params().use_map && msg->data)
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
    if (lctlPtr->get_params().use_map)
    {
        // 对全局地图排除较远的点
        for (int i = 0; i < terrainMapRecord_pcl->points.size(); i++)
        {
            point = terrainMapRecord_pcl->points[i];
            float pointX = point.x;
            float pointY = point.y;
            // 计算点云到车体的距离
            float dis = sqrt((pointX - vehicleX) * (pointX - vehicleX) + (pointY - vehicleY) * (pointY - vehicleY));

            if (dis < pathScale * lctlPtr->get_params().adjacentRange + 0.5)
            {
                terrainCloudCrop->push_back(point);
            }
        }
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
        if ((dis < pathScale * lctlPtr->get_params().adjacentRange + 0.3) && (point.intensity > lctlPtr->get_params().obstacleHeightThre || lctlPtr->get_params().useCost))
        {
            terrainCloudCrop->push_back(point);
        }
    }

    pcl::PointCloud<pcl::PointXYZI> terrainAddPoints;
    terrainAddPoints.clear();

    // 遍历点云插值
    // 先做一次降采样
    terrainCloudDwz->clear();
    terrainDwzFilter.setInputCloud(terrainCloudCrop);
    terrainDwzFilter.filter(*terrainCloudDwz);
    *terrainCloudCrop = *terrainCloudDwz;
    for (int i = 0; i < terrainCloudCrop->points.size(); i++)
    {
        // 计算点云到车体的距离
        float dis = sqrt((terrainCloudCrop->points[i].x - vehicleX) * (terrainCloudCrop->points[i].x - vehicleX) + (terrainCloudCrop->points[i].y - vehicleY) * (terrainCloudCrop->points[i].y - vehicleY));
        if(lctlPtr->get_params().add_point_radius > 0.01 || dis >= 1.5)
        {
            double add_point_radius_auc = lctlPtr->get_params().add_point_radius;
            if(dis >= 1.5)
            {
                add_point_radius_auc = lctlPtr->get_params().add_point_radius + dis * lctlPtr->get_params().add_point_radius_far;
            }
            // 45度插值一次
            for (int j = 1; j < 8; j++)
            {
                point.x = terrainCloudCrop->points[i].x + add_point_radius_auc * cos(j * 45 * M_PI / 180);
                point.y = terrainCloudCrop->points[i].y + add_point_radius_auc * sin(j * 45 * M_PI / 180);
                point.z = terrainCloudCrop->points[i].z;
                point.intensity = lctlPtr->get_params().obstacleHeightThre + 0.1;
                terrainCloudDwz->push_back(point);
                terrainAddPoints.push_back(point);
            }
        }
    }
    *terrainCloudCrop = *terrainCloudDwz;
    // 发布插值点云
    sensor_msgs::PointCloud2 terrainAddPoints2;
    pcl::toROSMsg(terrainAddPoints, terrainAddPoints2);
    terrainAddPoints2.header.stamp = ros::Time::now();
    terrainAddPoints2.header.frame_id = "map";
    pubAddPoints.publish(terrainAddPoints2);
    terrainAddPoints.clear();

    //  最后做一次降采样
    terrainCloudDwz->clear();
    terrainDwzFilter.setInputCloud(terrainCloudCrop);
    terrainDwzFilter.filter(*terrainCloudDwz);
    newTerrainCloud = true;
    plannerCloud->clear();
    *plannerCloud = *terrainCloudDwz;
}

void LpNode::globalPointHandler(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    goal_point = *msg;
}