#include "start_control/start_control.h"

StartControl::StartControl() : lidar_points(new pcl::PointCloud<pcl::PointXYZI>())
{
    this->rplidar_status.data = false;
    this->lidar_points->clear();
    rplidar_status_sub = nh.subscribe<std_msgs::Bool>("/rplidar_status", 10, &StartControl::rplidarStatusCallBack, this);
    rplidar_sub = nh.subscribe<sensor_msgs::PointCloud2>("/laser_point", 10, &StartControl::rplidarCallBack, this);
}

// 检测是否收到2D雷达数据
void StartControl::rplidarCallBack(const sensor_msgs::PointCloud2ConstPtr &addPoints)
{
    if (lidar_points->size() < 50)
    {
        pcl::PointCloud<pcl::PointXYZI> pcl_temp;
        pcl::fromROSMsg(*addPoints, pcl_temp);
        for (int i = 0; i < pcl_temp.size(); i++)
        {
            this->lidar_points->push_back(pcl_temp[i]);
        }
    }
}

void StartControl::rplidarStatusCallBack(const std_msgs::BoolConstPtr &status)
{
    this->rplidar_status = *status;
}
