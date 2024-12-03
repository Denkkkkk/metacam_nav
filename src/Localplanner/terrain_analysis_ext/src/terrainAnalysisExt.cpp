#include <math.h>
#include <queue>
#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

/*
  1. 加入车体中心向外扩展连接片区的墙面边界判断,这样的独立的方法
  2. 融合上/terrain_map的分析结果
  3. 加入一个局部地形地图分析的半径限制,默认为4m
  4. 新发布的融合地形分析话题为/terrain_map_ext
*/

using namespace std;

const double PI = 3.1415926;

double scanVoxelSize = 0.1;
double decayTime = 10.0;
double noDecayDis = 0;
double clearingDis = 30.0;
bool clearingCloud = false;
bool useSorting = false;
double quantileZ = 0.25;
double vehicleHeight = 1.5;
int voxelPointUpdateThre = 100;
double voxelTimeUpdateThre = 2.0;
double lowerBoundZ = -1.5;
double upperBoundZ = 1.0;
double disRatioZ = 0.1;
bool checkTerrainConn = true;       // 是否进行地形连接性检查
double terrainUnderVehicle = -0.75; // 车辆下方的地形高度，用于设置中心平面体素的默认高程
double terrainConnThre = 0.5;       // 地形连接的高程差异阈值。如果相邻平面体素的高程差异小于此阈值，它们将被认为是连接的
double ceilingFilteringThre = 2.0;  // 天花板过滤的高程差异阈值。大于这个阈值认为是天花板,不作为连接
double localTerrainMapRadius = 4.0; // 局部地形图的半径。这个参数可能用于限制地形连接性检查的范围，确保仅在一定半径范围内进行检查。

// terrain voxel parameters
// 体素的尺寸和数量都比一般的地形分析翻倍了
float terrainVoxelSize = 2.0;
int terrainVoxelShiftX = 0;
int terrainVoxelShiftY = 0;
const int terrainVoxelWidth = 41;
int terrainVoxelHalfWidth = (terrainVoxelWidth - 1) / 2;
const int terrainVoxelNum = terrainVoxelWidth * terrainVoxelWidth;

// planar voxel parameters
float planarVoxelSize = 0.4;
const int planarVoxelWidth = 101;
int planarVoxelHalfWidth = (planarVoxelWidth - 1) / 2;
const int planarVoxelNum = planarVoxelWidth * planarVoxelWidth;

pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloud(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCrop(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudDwz(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr terrainCloud(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr terrainCloudElev(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr terrainCloudLocal(new pcl::PointCloud<pcl::PointXYZI>()); // ROSMsg转成pcl格式后的地形点云/terrain_map
pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloud[terrainVoxelNum];

int terrainVoxelUpdateNum[terrainVoxelNum] = {0};
float terrainVoxelUpdateTime[terrainVoxelNum] = {0};
float planarVoxelElev[planarVoxelNum] = {0};
int planarVoxelConn[planarVoxelNum] = {0};
vector<float> planarPointElev[planarVoxelNum];
queue<int> planarVoxelQueue;

double laserCloudTime = 0;
bool newlaserCloud = false;

double systemInitTime = 0;
bool systemInited = false;

float vehicleRoll = 0, vehiclePitch = 0, vehicleYaw = 0;
float vehicleX = 0, vehicleY = 0, vehicleZ = 0;

pcl::VoxelGrid<pcl::PointXYZI> downSizeFilter;
pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
/*
  创建一个基于点类型 pcl::PointXYZI 的 kd-tree 对象。这个对象可以用于执行各种点云搜索操作，如最近邻搜索等.
  kd-tree 是一种用于高效搜索最近邻点的数据结构，特别适用于处理三维点云数据。
*/

// state estimation callback function
// 用odom更新车体位姿
void odometryHandler(const nav_msgs::Odometry::ConstPtr &odom)
{
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

// registered laser scan callback function
// 与地形分析一致
void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloud2)
{
    laserCloudTime = laserCloud2->header.stamp.toSec();

    if (!systemInited)
    {
        systemInitTime = laserCloudTime;
        systemInited = true;
    }

    laserCloud->clear();
    pcl::fromROSMsg(*laserCloud2, *laserCloud);

    pcl::PointXYZI point;
    laserCloudCrop->clear();
    int laserCloudSize = laserCloud->points.size();
    for (int i = 0; i < laserCloudSize; i++)
    {
        point = laserCloud->points[i];

        float pointX = point.x;
        float pointY = point.y;
        float pointZ = point.z;

        float dis = sqrt((pointX - vehicleX) * (pointX - vehicleX) + (pointY - vehicleY) * (pointY - vehicleY));
        if (pointZ - vehicleZ > lowerBoundZ - disRatioZ * dis && // disRatioZ为点云处理的高度与距离的比例， 点云越近点云就得越高,当距离dis=0时点云相对车体高恰好在min/maxRelZ之间,点云越远这个约束越无效
            pointZ - vehicleZ < upperBoundZ + disRatioZ * dis &&
            dis < terrainVoxelSize * (terrainVoxelHalfWidth + 1)) // 不超出体素区域
        {
            point.x = pointX;
            point.y = pointY;
            point.z = pointZ;
            point.intensity = laserCloudTime - systemInitTime;
            laserCloudCrop->push_back(point);
        }
    }

    newlaserCloud = true;
}

// local terrain cloud callback function
void terrainCloudLocalHandler(const sensor_msgs::PointCloud2ConstPtr &terrainCloudLocal2)
{
    terrainCloudLocal->clear();
    pcl::fromROSMsg(*terrainCloudLocal2, *terrainCloudLocal);
}

// cloud clearing callback function
void clearingHandler(const std_msgs::Float32::ConstPtr &dis)
{
    clearingDis = dis->data;
    clearingCloud = true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "terrainAnalysisExt");
    ros::NodeHandle nh;
    ros::NodeHandle nhPrivate = ros::NodeHandle("~");

    nhPrivate.getParam("scanVoxelSize", scanVoxelSize);
    nhPrivate.getParam("decayTime", decayTime);
    nhPrivate.getParam("noDecayDis", noDecayDis);
    nhPrivate.getParam("clearingDis", clearingDis);
    nhPrivate.getParam("useSorting", useSorting);
    nhPrivate.getParam("quantileZ", quantileZ);
    nhPrivate.getParam("vehicleHeight", vehicleHeight);
    nhPrivate.getParam("voxelPointUpdateThre", voxelPointUpdateThre);
    nhPrivate.getParam("voxelTimeUpdateThre", voxelTimeUpdateThre);
    nhPrivate.getParam("lowerBoundZ", lowerBoundZ);
    nhPrivate.getParam("upperBoundZ", upperBoundZ);
    nhPrivate.getParam("disRatioZ", disRatioZ);
    nhPrivate.getParam("checkTerrainConn", checkTerrainConn);
    nhPrivate.getParam("terrainUnderVehicle", terrainUnderVehicle);
    nhPrivate.getParam("terrainConnThre", terrainConnThre);
    nhPrivate.getParam("ceilingFilteringThre", ceilingFilteringThre);
    nhPrivate.getParam("localTerrainMapRadius", localTerrainMapRadius);

    ros::Subscriber subOdometry = nh.subscribe<nav_msgs::Odometry>("/Odometry", 5, odometryHandler);
    ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/cloud_interface", 5, laserCloudHandler);
    ros::Subscriber subClearing = nh.subscribe<std_msgs::Float32>("/cloud_clearing", 5, clearingHandler);
    // 主要就是这里,对那边地形点云图的进一步处理
    ros::Subscriber subTerrainCloudLocal = nh.subscribe<sensor_msgs::PointCloud2>("/terrain_map", 2, terrainCloudLocalHandler);
    ros::Publisher pubTerrainCloud = nh.advertise<sensor_msgs::PointCloud2>("/terrain_map_ext", 2);
    for (int i = 0; i < terrainVoxelNum; i++)
    {
        terrainVoxelCloud[i].reset(new pcl::PointCloud<pcl::PointXYZI>());
    }
    downSizeFilter.setLeafSize(scanVoxelSize, scanVoxelSize, scanVoxelSize);
    std::vector<int> pointIdxNKNSearch;
    std::vector<float> pointNKNSquaredDistance;
    ros::Rate rate(100);
    bool status = ros::ok();
    while (status)
    {
        ros::spinOnce();

        if (newlaserCloud)
        {
            newlaserCloud = false;

            // terrain voxel roll over\
            // 同样是先回滚地形中心与车体位置一致
            float terrainVoxelCenX = terrainVoxelSize * terrainVoxelShiftX;
            float terrainVoxelCenY = terrainVoxelSize * terrainVoxelShiftY;

            while (vehicleX - terrainVoxelCenX < -terrainVoxelSize)
            {
                for (int indY = 0; indY < terrainVoxelWidth; indY++)
                {
                    pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloudPtr =
                        terrainVoxelCloud[terrainVoxelWidth * (terrainVoxelWidth - 1) + indY];
                    for (int indX = terrainVoxelWidth - 1; indX >= 1; indX--)
                    {
                        terrainVoxelCloud[terrainVoxelWidth * indX + indY] =
                            terrainVoxelCloud[terrainVoxelWidth * (indX - 1) + indY];
                    }
                    terrainVoxelCloud[indY] = terrainVoxelCloudPtr;
                    terrainVoxelCloud[indY]->clear();
                }
                terrainVoxelShiftX--;
                terrainVoxelCenX = terrainVoxelSize * terrainVoxelShiftX;
            }

            while (vehicleX - terrainVoxelCenX > terrainVoxelSize)
            {
                for (int indY = 0; indY < terrainVoxelWidth; indY++)
                {
                    pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloudPtr = terrainVoxelCloud[indY];
                    for (int indX = 0; indX < terrainVoxelWidth - 1; indX++)
                    {
                        terrainVoxelCloud[terrainVoxelWidth * indX + indY] =
                            terrainVoxelCloud[terrainVoxelWidth * (indX + 1) + indY];
                    }
                    terrainVoxelCloud[terrainVoxelWidth * (terrainVoxelWidth - 1) + indY] = terrainVoxelCloudPtr;
                    terrainVoxelCloud[terrainVoxelWidth * (terrainVoxelWidth - 1) + indY]->clear();
                }
                terrainVoxelShiftX++;
                terrainVoxelCenX = terrainVoxelSize * terrainVoxelShiftX;
            }

            while (vehicleY - terrainVoxelCenY < -terrainVoxelSize)
            {
                for (int indX = 0; indX < terrainVoxelWidth; indX++)
                {
                    pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloudPtr =
                        terrainVoxelCloud[terrainVoxelWidth * indX + (terrainVoxelWidth - 1)];
                    for (int indY = terrainVoxelWidth - 1; indY >= 1; indY--)
                    {
                        terrainVoxelCloud[terrainVoxelWidth * indX + indY] =
                            terrainVoxelCloud[terrainVoxelWidth * indX + (indY - 1)];
                    }
                    terrainVoxelCloud[terrainVoxelWidth * indX] = terrainVoxelCloudPtr;
                    terrainVoxelCloud[terrainVoxelWidth * indX]->clear();
                }
                terrainVoxelShiftY--;
                terrainVoxelCenY = terrainVoxelSize * terrainVoxelShiftY;
            }

            while (vehicleY - terrainVoxelCenY > terrainVoxelSize)
            {
                for (int indX = 0; indX < terrainVoxelWidth; indX++)
                {
                    pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloudPtr = terrainVoxelCloud[terrainVoxelWidth * indX];
                    for (int indY = 0; indY < terrainVoxelWidth - 1; indY++)
                    {
                        terrainVoxelCloud[terrainVoxelWidth * indX + indY] =
                            terrainVoxelCloud[terrainVoxelWidth * indX + (indY + 1)];
                    }
                    terrainVoxelCloud[terrainVoxelWidth * indX + (terrainVoxelWidth - 1)] = terrainVoxelCloudPtr;
                    terrainVoxelCloud[terrainVoxelWidth * indX + (terrainVoxelWidth - 1)]->clear();
                }
                terrainVoxelShiftY++;
                terrainVoxelCenY = terrainVoxelSize * terrainVoxelShiftY;
            }

            // stack registered laser scans
            pcl::PointXYZI point;
            int laserCloudCropSize = laserCloudCrop->points.size();

            // 同样是先将原始点云压入对应的地形体素网格位置
            for (int i = 0; i < laserCloudCropSize; i++)
            {
                point = laserCloudCrop->points[i];

                int indX = int((point.x - vehicleX + terrainVoxelSize / 2) / terrainVoxelSize) + terrainVoxelHalfWidth;
                int indY = int((point.y - vehicleY + terrainVoxelSize / 2) / terrainVoxelSize) + terrainVoxelHalfWidth;

                if (point.x - vehicleX + terrainVoxelSize / 2 < 0)
                    indX--;
                if (point.y - vehicleY + terrainVoxelSize / 2 < 0)
                    indY--;

                if (indX >= 0 && indX < terrainVoxelWidth && indY >= 0 && indY < terrainVoxelWidth)
                {
                    terrainVoxelCloud[terrainVoxelWidth * indX + indY]->push_back(point);
                    terrainVoxelUpdateNum[terrainVoxelWidth * indX + indY]++;
                }
            }

            // 同样是三个条件下采样和筛选terrainVoxelCloud到terrainVoxelCloudPtr
            for (int ind = 0; ind < terrainVoxelNum; ind++)
            {
                if (terrainVoxelUpdateNum[ind] >= voxelPointUpdateThre ||
                    laserCloudTime - systemInitTime - terrainVoxelUpdateTime[ind] >= voxelTimeUpdateThre || clearingCloud)
                {
                    pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloudPtr = terrainVoxelCloud[ind];

                    laserCloudDwz->clear();
                    downSizeFilter.setInputCloud(terrainVoxelCloudPtr);
                    downSizeFilter.filter(*laserCloudDwz);

                    terrainVoxelCloudPtr->clear();
                    int laserCloudDwzSize = laserCloudDwz->points.size();
                    for (int i = 0; i < laserCloudDwzSize; i++)
                    {
                        point = laserCloudDwz->points[i];
                        float dis = sqrt((point.x - vehicleX) * (point.x - vehicleX) + (point.y - vehicleY) * (point.y - vehicleY));
                        if (point.z - vehicleZ > lowerBoundZ - disRatioZ * dis &&
                            point.z - vehicleZ < upperBoundZ + disRatioZ * dis &&
                            (laserCloudTime - systemInitTime - point.intensity < decayTime || dis < noDecayDis) &&
                            !(dis < clearingDis && clearingCloud))
                        {
                            terrainVoxelCloudPtr->push_back(point);
                        }
                    }

                    terrainVoxelUpdateNum[ind] = 0;
                    terrainVoxelUpdateTime[ind] = laserCloudTime - systemInitTime;
                }
            }

            // 从这里开始出现区别,取的是车体周围20*20的区域,同样是翻倍处理
            terrainCloud->clear();
            for (int indX = terrainVoxelHalfWidth - 10; indX <= terrainVoxelHalfWidth + 10; indX++)
            {
                for (int indY = terrainVoxelHalfWidth - 10; indY <= terrainVoxelHalfWidth + 10; indY++)
                {
                    *terrainCloud += *terrainVoxelCloud[terrainVoxelWidth * indX + indY];
                }
            }

            // estimate ground and compute elevation for each point
            for (int i = 0; i < planarVoxelNum; i++)
            {
                planarVoxelElev[i] = 0;
                planarVoxelConn[i] = 0;
                planarPointElev[i].clear();
            }

            // 同样是一个点云高程影响周围3*3区域,但是**高程上下限限制没有了**
            int terrainCloudSize = terrainCloud->points.size();
            for (int i = 0; i < terrainCloudSize; i++)
            {
                point = terrainCloud->points[i];
                float dis = sqrt((point.x - vehicleX) * (point.x - vehicleX) + (point.y - vehicleY) * (point.y - vehicleY));
                if (point.z - vehicleZ > lowerBoundZ - disRatioZ * dis && point.z - vehicleZ < upperBoundZ + disRatioZ * dis)
                {
                    int indX = int((point.x - vehicleX + planarVoxelSize / 2) / planarVoxelSize) + planarVoxelHalfWidth;
                    int indY = int((point.y - vehicleY + planarVoxelSize / 2) / planarVoxelSize) + planarVoxelHalfWidth;

                    if (point.x - vehicleX + planarVoxelSize / 2 < 0)
                        indX--;
                    if (point.y - vehicleY + planarVoxelSize / 2 < 0)
                        indY--;

                    for (int dX = -1; dX <= 1; dX++)
                    {
                        for (int dY = -1; dY <= 1; dY++)
                        {
                            if (indX + dX >= 0 && indX + dX < planarVoxelWidth && indY + dY >= 0 && indY + dY < planarVoxelWidth)
                            {
                                planarPointElev[planarVoxelWidth * (indX + dX) + indY + dY].push_back(point.z);
                            }
                        }
                    }
                }
            }

            // 这里的代表高程点选取还是一样
            if (useSorting)
            {
                for (int i = 0; i < planarVoxelNum; i++)
                {
                    int planarPointElevSize = planarPointElev[i].size();
                    if (planarPointElevSize > 0)
                    {
                        sort(planarPointElev[i].begin(), planarPointElev[i].end());

                        int quantileID = int(quantileZ * planarPointElevSize);
                        if (quantileID < 0)
                            quantileID = 0;
                        else if (quantileID >= planarPointElevSize)
                            quantileID = planarPointElevSize - 1;

                        planarVoxelElev[i] = planarPointElev[i][quantileID];
                    }
                }
            }
            else
            {
                for (int i = 0; i < planarVoxelNum; i++)
                {
                    int planarPointElevSize = planarPointElev[i].size();
                    if (planarPointElevSize > 0)
                    {
                        float minZ = 1000.0;
                        int minID = -1;
                        for (int j = 0; j < planarPointElevSize; j++)
                        {
                            if (planarPointElev[i][j] < minZ)
                            {
                                minZ = planarPointElev[i][j];
                                minID = j;
                            }
                        }

                        if (minID != -1)
                        {
                            planarVoxelElev[i] = planarPointElev[i][minID];
                        }
                    }
                }
            }

            // check terrain connectivity to remove ceiling
            // 关键部分1,执行地形连接性检查,其实就是查找和车体当前地形高度一致(连接)的一大片区域
            if (checkTerrainConn)
            {
                int ind = planarVoxelWidth * planarVoxelHalfWidth + planarVoxelHalfWidth; // 定位到中心平面体素
                if (planarPointElev[ind].size() == 0)                                     // 检查高程是否为空
                    planarVoxelElev[ind] = vehicleZ + terrainUnderVehicle;                // 空的话设置成这个默认值

                planarVoxelQueue.push(ind); // 中心平面体素的索引加入队列 (planarVoxelQueue)，标记其连接状态1个连接(自己)
                planarVoxelConn[ind] = 1;
                while (!planarVoxelQueue.empty())
                {
                    int front = planarVoxelQueue.front(); // 第一轮只有中心体素
                    planarVoxelConn[front] = 2;           // 若只作为连接性检查的中心,不被检查到的话为2(即边界点),否则一般连接为1
                    planarVoxelQueue.pop();               // 移除队首元素

                    int indX = int(front / planarVoxelWidth);
                    int indY = front % planarVoxelWidth;
                    for (int dX = -10; dX <= 10; dX++) // 遍历front周围一圈20*20内的体素网格,找出可认为连接的
                    {
                        for (int dY = -10; dY <= 10; dY++)
                        {
                            if (indX + dX >= 0 && indX + dX < planarVoxelWidth && indY + dY >= 0 && indY + dY < planarVoxelWidth)
                            {
                                ind = planarVoxelWidth * (indX + dX) + indY + dY;
                                if (planarVoxelConn[ind] == 0 && planarPointElev[ind].size() > 0) // 如果这个连接位置连接性为0,即没有进行过连接检查
                                {
                                    if (fabs(planarVoxelElev[front] - planarVoxelElev[ind]) < terrainConnThre) // 连接阈值,连接的先置位,压进来稍后以它为中心继续找连接
                                    {
                                        planarVoxelQueue.push(ind);
                                        planarVoxelConn[ind] = 1;
                                    }
                                    else if (fabs(planarVoxelElev[front] - planarVoxelElev[ind]) > ceilingFilteringThre)
                                    {
                                        planarVoxelConn[ind] = -1;
                                    }
                                }
                            }
                        }
                    }
                }
            }

            // compute terrain map beyond localTerrainMapRadius
            // 同样的确认有效墙面阻塞点,
            terrainCloudElev->clear();
            int terrainCloudElevSize = 0;
            for (int i = 0; i < terrainCloudSize; i++)
            {
                point = terrainCloud->points[i];
                float dis = sqrt((point.x - vehicleX) * (point.x - vehicleX) + (point.y - vehicleY) * (point.y - vehicleY));
                if (point.z - vehicleZ > lowerBoundZ - disRatioZ * dis &&
                    point.z - vehicleZ < upperBoundZ + disRatioZ * dis && // 只做近处的点云高度限制,越近越要在(lowerBoundZ,upperBoundZ)范围内
                    dis > localTerrainMapRadius)                          // 地形图分析的半径范围限制
                {
                    int indX = int((point.x - vehicleX + planarVoxelSize / 2) / planarVoxelSize) + planarVoxelHalfWidth;
                    int indY = int((point.y - vehicleY + planarVoxelSize / 2) / planarVoxelSize) + planarVoxelHalfWidth;

                    if (point.x - vehicleX + planarVoxelSize / 2 < 0)
                        indX--;
                    if (point.y - vehicleY + planarVoxelSize / 2 < 0)
                        indY--;

                    if (indX >= 0 && indX < planarVoxelWidth && indY >= 0 && indY < planarVoxelWidth) // 防止越界
                    {
                        int ind = planarVoxelWidth * indX + indY;
                        float disZ = fabs(point.z - planarVoxelElev[ind]);
                        if (disZ < vehicleHeight &&                           // 遮挡在车体高度范围内,不设置下方的阈值,默认连接片区边界就是墙面
                            (planarVoxelConn[ind] == 2 || !checkTerrainConn)) // 作为车体延伸除去的连接片区的边界才置为墙面
                        {
                            terrainCloudElev->push_back(point);
                            terrainCloudElev->points[terrainCloudElevSize].x = point.x;
                            terrainCloudElev->points[terrainCloudElevSize].y = point.y;
                            terrainCloudElev->points[terrainCloudElevSize].z = point.z;
                            terrainCloudElev->points[terrainCloudElevSize].intensity = disZ;

                            terrainCloudElevSize++;
                        }
                    }
                }
            }

            // merge in local terrain map within localTerrainMapRadius
            // 前面只是进行车体中心向外扩展连接片区的墙面边界判断,是独立的方法
            // 现在将/terrain_map的研究融合进来
            int terrainCloudLocalSize = terrainCloudLocal->points.size();
            for (int i = 0; i < terrainCloudLocalSize; i++)
            {
                point = terrainCloudLocal->points[i];
                float dis = sqrt((point.x - vehicleX) * (point.x - vehicleX) + (point.y - vehicleY) * (point.y - vehicleY));
                if (dis <= localTerrainMapRadius) // 默认/terrain_map分析正确,只是做局部地图半径控制
                {
                    terrainCloudElev->push_back(point);
                }
            }

            clearingCloud = false;

            // publish points with elevation
            sensor_msgs::PointCloud2 terrainCloud2;
            pcl::toROSMsg(*terrainCloudElev, terrainCloud2);
            terrainCloud2.header.stamp = ros::Time().fromSec(laserCloudTime);
            terrainCloud2.header.frame_id = "map";
            pubTerrainCloud.publish(terrainCloud2);
        }

        status = ros::ok();
        rate.sleep();
    }

    return 0;
}
