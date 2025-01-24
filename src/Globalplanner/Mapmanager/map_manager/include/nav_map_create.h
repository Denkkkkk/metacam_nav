/**
 * @file map_create.h
 * @author 李东权 1327165187@qq.com
 * @brief
 * @version 1.0
 * @date 2024-10-21
 *
 * @copyright Copyright (c) 2024
 *
 */

#include <math.h>
#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h> //pcd 读写类相关的头文件。
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <chrono>

#define CREATE_STATIC()   \
    do                    \
    {                     \
        static int i = 1; \
        if (i++ < 20)     \
        {                 \
            return;       \
        }                 \
        i = 1;            \
    } while (false)

class MapRecord
{
private:
    ros::NodeHandle nh;
    ros::NodeHandle nhPrivate = ros::NodeHandle("~");
    ros::Subscriber terrain_map_sub;
    ros::Subscriber cloud_map_sub;

    pcl::VoxelGrid<pcl::PointXYZI> terrainRecordFilter; // 生成滤波器
    // pcl::PointCloud是模板类，pcl::PointXYZI是实际类型，()是默认构造
    pcl::PointCloud<pcl::PointXYZI>::Ptr terrain_map_xyzi;

    float leaf_size = 0.1;
    std::stringstream terrain_map_ss;
    std::stringstream terrain_map_ss_temp;
    std::stringstream relocalization_map_ss;
    std::stringstream relocalization_map_ss_temp;

    void terrainMapCallback(const sensor_msgs::PointCloud2::ConstPtr &terrain_map_pc2);
    void cloudMapCallback(const sensor_msgs::PointCloud2::ConstPtr &cloud_map_pc2);
    void save_pcd(const sensor_msgs::PointCloud2 &cloud, std::stringstream& file_name, bool binary_ = false);

public:
    MapRecord();
    ~MapRecord() {};

    pcl::PointCloud<pcl::PointXYZI>::Ptr terrain_map_record; // 记录的地形点云图
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_map_record; // 记录的cloud点云图
    ros::Publisher terrain_map_pub;
    ros::Publisher cloud_map_pub;
    ros::Publisher map_flie_pub;
    bool record = false;
    std::string pcd_path;
    double obstacleHeightThre = 0.12;
    bool save_always = false;
    std::string visual_pcd_path;
};