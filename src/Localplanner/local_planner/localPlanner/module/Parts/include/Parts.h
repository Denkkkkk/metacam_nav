#pragma once
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float32.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
using namespace std;

#ifndef PLOTPATHSET
#define PLOTPATHSET 1 // 开启可行路径可视化
#endif
extern const double PI;
extern string pathFolder;
extern double terrainVoxelSize;
extern bool checkRotObstacle;
extern const int laserCloudStackNum;
extern int laserCloudCount;
extern double maxSpeed;
extern double goalX;
extern double goalY;

extern float joySpeed; // 车体速度当前值
extern double joyDir;  // joyDir为车体到目标点之间的yaw角

extern const int pathNum; // 一个10度方向待规划的路径数
extern const int groupNum;
extern float gridVoxelSize;
extern float searchRadius;
extern float gridVoxelOffsetX;
extern float gridVoxelOffsetY;
extern const int gridVoxelNumX;
extern const int gridVoxelNumY;
extern const int gridVoxelNum;

extern int pathList[343];
extern float endDirPathList[343];            // 可能用于存储路径终点的方向信息
extern int clearPathList[36 * 343];          // 路径的清除标志位
extern float pathPenaltyList[36 * 343];      // 可能用于存储路径的惩罚值
extern float clearPathPerGroupScore[36 * 7]; // 能用于存储每个路径组的分辨率分数
extern bool newTerrainCloud;                 // 是否有新的地形云数据可用
extern float detourWei;

// 初始化时间戳
extern double odomTime;
// 初始化车体的位姿
extern float vehicleRoll, vehiclePitch, vehicleYaw;
extern float vehicleX, vehicleY, vehicleZ;

extern std::vector<int> correspondences[161 * 451]; // 用于网格和路径相对应

extern pcl::VoxelGrid<pcl::PointXYZI> laserDwzFilter, terrainDwzFilter; // 对激光雷达数据和地形数据进行体素化滤波，后的点云数据存储
extern pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloud;                 //!  存储激光雷达扫描数据
extern pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCrop;             //! 存储经过裁剪的激光雷达扫描数据
extern pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudDwz;              //! 存储经过某种处理的激光雷达扫描数据

extern pcl::PointCloud<pcl::PointXYZI>::Ptr addPointsterrainCloud; // 存储地形相关的点云数据
extern pcl::PointCloud<pcl::PointXYZI>::Ptr terrainCloud;          // 存储地形相关的点云数据

extern pcl::PointCloud<pcl::PointXYZI>::Ptr terrainCloudCrop;
extern pcl::PointCloud<pcl::PointXYZI>::Ptr terrainCloudDwz;

extern pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudStack[1]; // 包含 laserCloudStackNum 个指向类型为 pcl::PointXYZI 的点云对象的指针。
extern pcl::PointCloud<pcl::PointXYZI>::Ptr plannerCloud;       // 可能用于存储路径规划相关的点云数据
extern pcl::PointCloud<pcl::PointXYZI>::Ptr plannerCloudCrop;
extern pcl::PointCloud<pcl::PointXYZI>::Ptr PannerAtuCloud;

extern pcl::PointCloud<pcl::PointXYZI>::Ptr addedObstacles;     // 存储被添加的障碍物的数据
extern pcl::PointCloud<pcl::PointXYZI>::Ptr addedObstaclesCrop; // 裁剪距离后的额加障碍物数据

extern pcl::PointCloud<pcl::PointXYZ>::Ptr startPaths[7]; // 存储不同路径组的起始路径或轨迹数据

#if PLOTPATHSET == 1
extern pcl::PointCloud<pcl::PointXYZI>::Ptr paths[343]; // 用于存储不同路径的数据
extern pcl::PointCloud<pcl::PointXYZI>::Ptr freePaths;  // 存储所有可行路径的数据
#endif

/**
 * @author 李东权 (1327165187@qq.com)
 * @brief  类重构cmu的局部路径规划框架。以下参数不在23赛季参数手册中，类封装时引入
 * @version 1.0
 */
extern pcl::PointXYZI point; // 点云暂存值
extern int plannerCloudSize; // 待处理新增点云数量

extern float sinVehicleYaw;
extern float cosVehicleYaw;

extern bool pathFound;           // 找到最终规划出来的路径
extern double pathRange;         // 路径范围(黄色)
extern float relativeGoalDis;    // 目标点相对车体的距离
extern float rotAng;             // rotAng当前检索方向和车体的角度
extern float angDiff;            // rotAng叠加上joyDir
extern float minObsAngCW;        // minObsAngCW是考虑车体的直径和考虑侧方障碍物的情况下，车顺时针能旋转的最小角度
extern float minObsAngCCW;       // minObsAngCCW是是考虑车体的直径和考虑侧方障碍物的情况下，车逆时针能旋转的最小角度
extern float angOffset;          // angOffset为车体的拐角
extern float diameter;           // diameter为车体的半对角线长度
extern int plannerCloudCropSize; // plannerCloudCropSize为待处理的点云数量
extern std_msgs::Float32 slowDown;
extern float planRangeK;

/*
 * 读取ply文件，获取点云的采样个数
 */
int readPlyHeader(FILE *filePtr);

/*
 * 读取startPaths.ply文件，并将点云信息传入到startPaths中
 * startPaths的意义是第一次采样的路径点，可以理解成
 */
void readStartPaths();

/*
 * 读取paths.ply文件，把点云信息存入到paths中
 * paths的意义是路径点的集合，此函数就是在生成初始化的黄色路径点集合
 */
void readPaths();

/**
 * @brief 读取pathList.ply文件,把路径数据放入pathList和endDirPathList中
 * @target  从指定的 PLY 文件中读取路径列表数据，将数据关联到相关数组中，以便后续的路径规划和使用
                       实现流程和上一个路径数据读取基本一致
 */
void readPathList();

/**
 * @brief 读取correspondences.txt文件，把数据放进中correspondences中
 *         correspondences的意义是路径点和碰撞体素网格的索引关系
 */
void readCorrespondences();