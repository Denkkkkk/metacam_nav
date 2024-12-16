/**
 * @file terrainAnalysis.cpp
 * @author cmu
 * @brief 改进和带注释的地面分割算法
 * @version 2.0
 * @date 2023-12-9，2024-4-20
 *
 * @copyright Copyright (c) 2024
 *
 */
#include <math.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float32.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

/*
    100hz循环地形分析,叠加多帧点云,将机器人周围区域的地面进行可行驶区域分析,不能通行的区域会以点云的形式传输给localPlanner,用于localPlanner避障.

    开放接口（用户自己选择发布）：
    1 --- /map_clearing  可以一键清空地形点云地图，没有发布者，用户自己开发时可使用。当且仅当重开程序或这个清空时，noDataInited置0
    必须提供：
    1 --- /cloud_interface 由pointlio_interface或者loam_interface进行话题的重映射，lio模块必须提供这个原始扫描点云

    对外发布：
    1 --- /terrain_map, terrainCloudElev是最终分析出来的有效墙面遮挡(1.低于车体高度 2.高于地面高度 3.有效堵塞点云足够多)
                        terrainCloudElev相对/map节点,且转为pcl::ROSMsg改名terrainCloud2
*/
using namespace std;
const double PI = 3.1415926;
double scanVoxelSize = 0.05;  // 滤波器的叶子大小，这里XYZ方向叶子大小一致
double decayTime = 2.0;       // 衰减时间
double noDecayDis = 0.0;      // 无衰减距离，在这个范围内的点不会随时间衰减，不会被抹去
double clearingDis = 8.0;     // 手动清除时的清除半径外范围
bool clearingCloud = false;   // 是否刚清空完所有点云
bool useSorting = true;       // 是否使用排序
double quantileZ = 0.25;      // 小于1, 如果useSorting,对平面体素高程planarPointElev仅保留第size*quantileZ的高程信息,比如7个高程信息,7*0.25=1.75约1,即仅保留第二小的高程信息
bool considerDrop = false;    // 是否考虑车子下方的障碍物点云
bool limitGroundLift = false; // 是否限制最大地面高度
double maxGroundLift = 0.15;  // 最大地面高度
bool clearDyObs = false;
double minDyObsDis = 0.3;
double minDyObsAngle = 0;
double minDyObsRelZ = -0.5;  // 认为动态障碍物处于水平面的参考高度
double minDyObsVFOV = -16.0; // 动态障碍物垂直视场角最小值
double maxDyObsVFOV = 16.0;
int minDyObsPointNum = 1; // 最小动态障碍物点数
bool noDataObstacle = false;
int noDataBlockSkipNum = 0; // 无数据块跳过数量
int minBlockPointNum = 10;  // 堵塞阈值,某体素位置要求扫到的有效高程的数量
double vehicleHeight = 1.5;
int voxelPointUpdateThre = 100;
double voxelTimeUpdateThre = 2.0;
double minRelZ = -1.5;
double maxRelZ = 0.2;
double disRatioZ = 0.2; // 点云处理时，距离占比

// 地形体素
float terrainVoxelSize = 1.0; // 地形中每个体素的边长大小
int terrainVoxelShiftX = 0;   // 地形体素在 X 和 Y 方向的偏移值。
int terrainVoxelShiftY = 0;
const int terrainVoxelWidth = 21;                                  // 地形体素网格的一条边上有多少个网格
int terrainVoxelHalfWidth = (terrainVoxelWidth - 1) / 2;           // 体素网格半边长（以网格数为单位）
const int terrainVoxelNum = terrainVoxelWidth * terrainVoxelWidth; // 地形网格中体素的总数

// 平面体素
double planarVoxelSize = 0.2; // 平面中每个体素的大小
const int planarVoxelWidth = 101;
int planarVoxelHalfWidth = (planarVoxelWidth - 1) / 2;
const int planarVoxelNum = planarVoxelWidth * planarVoxelWidth;

pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloud(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloud_last(new pcl::PointCloud<pcl::PointXYZI>());

pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCrop(new pcl::PointCloud<pcl::PointXYZI>()); // 回调函数筛选后的最原始点云
pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudDwz(new pcl::PointCloud<pcl::PointXYZI>());  // 下采样后

pcl::PointCloud<pcl::PointXYZI>::Ptr terrainCloud(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr terrainCloudElev(new pcl::PointCloud<pcl::PointXYZI>()); // 存储最终地形分析完的被认为是墙面障碍的点
pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloud[terrainVoxelNum];                      // 智能指针,地形体素和对应初始的点云,更新(下采样和筛选)后还是他来存

int terrainVoxelUpdateNum[terrainVoxelNum] = {0};    // 地形体素待更新的点云数量,表示刚刚进来体素网格的点云为未下采样和筛选
float terrainVoxelUpdateTime[terrainVoxelNum] = {0}; // 地形体素的最后一次更新时间，以laserCloudTime - systemInitTime方式写入，为相对初始时刻的相对时间

float planarVoxelElev[planarVoxelNum] = {0}; // 平面体素的最终高程信息,保留为最小值或开启userSorting后为从小到大筛选的代表性点云高度
// (相比于上面,是不稳定的暂存值)所有处理过的，在有效高度范围内的高程（点云高度）信息，即point.z,一个体素位置可能有多个高程z。
// 以下这已经是一个矩阵
vector<float> planarPointElev[planarVoxelNum];
int planarVoxelEdge[planarVoxelNum] = {0};  // 标记平面体素是否位于地形的边缘，有多少个点云是边缘点
int planarVoxelDyObs[planarVoxelNum] = {0}; // 标记平面体素是否包含动态障碍物，有多少个点云被认为是动态的

double laserCloudTime = 0; // 该帧点云的时间戳
bool newlaserCloud = false;

double systemInitTime = 0;
bool systemInited = false;
int noDataInited = 0;

float vehicleRoll = 0, vehiclePitch = 0, vehicleYaw = 0;
float vehicleX = 0, vehicleY = 0, vehicleZ = 0;
float vehicleXRec = 0, vehicleYRec = 0;

float sinVehicleRoll = 0, cosVehicleRoll = 0;
float sinVehiclePitch = 0, cosVehiclePitch = 0;
float sinVehicleYaw = 0, cosVehicleYaw = 0;

double terrainMapRadius = 10.0; // 地形分析的半径

pcl::VoxelGrid<pcl::PointXYZI> downSizeFilter; // 下采样滤波器
/*
    pcl::VoxelGrid 是点云库（Point Cloud Library，PCL）中的一个类，用于进行点云体素网格滤波（Voxel Grid Filtering）。
    体素网格滤波是一种降采样方法，通过将点云划分为规则的三维体素网格，
    每个体素内只保留一个点或使用体素内点的平均值表示，从而减少点云的数据量。
*/

// state estimation callback function
// 根据里程计消息更新车体的位姿，主打一个车体的状态估计。同时记录车体的初始位置，
// 当车体的位置偏离初始位置足够远时，开启处理地形数据边缘信息的标志位
void odometryHandler(const nav_msgs::Odometry::ConstPtr &odom)
{
    double roll, pitch, yaw;
    geometry_msgs::Quaternion geoQuat = odom->pose.pose.orientation;
    tf::Matrix3x3(tf::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w))
        .getRPY(roll, pitch, yaw);

    vehicleRoll = roll;
    vehiclePitch = pitch;
    vehicleYaw = yaw;
    vehicleX = odom->pose.pose.position.x;
    vehicleY = odom->pose.pose.position.y;
    vehicleZ = odom->pose.pose.position.z;

    sinVehicleRoll = sin(vehicleRoll);
    cosVehicleRoll = cos(vehicleRoll);
    sinVehiclePitch = sin(vehiclePitch);
    cosVehiclePitch = cos(vehiclePitch);
    sinVehicleYaw = sin(vehicleYaw);
    cosVehicleYaw = cos(vehicleYaw);

    // 使用vehicleXRec、vehicleYRec记录下首次运行时的位置
    if (noDataInited == 0)
    {
        vehicleXRec = vehicleX;
        vehicleYRec = vehicleY;
        noDataInited = 1;
    }
    if (noDataInited == 1)
    {
        float dis = sqrt((vehicleX - vehicleXRec) * (vehicleX - vehicleXRec) +
                         (vehicleY - vehicleYRec) * (vehicleY - vehicleYRec));
        if (dis >= noDecayDis)
            noDataInited = 2;
    }
}

// registered laser scan callback function
// 这个话题的点云是lio输出的点云，当新一帧点云到来时进入点云处理
// 限制点云高程，在距离范围内设置点云随时间衰减
void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloud2)
{
    // 记录该点云的时间戳
    laserCloudTime = laserCloud2->header.stamp.toSec();
    // 记录第一次收到点云的时间点，以此为初始时刻，systemInited整个程序只跑一遍
    if (!systemInited)
    {
        systemInitTime = laserCloudTime;
        systemInited = true;
    }
    // 清除上一次处理过的点云，解放变量
    laserCloud->clear();
    // 将当前点云转换成ROS格式
    pcl::fromROSMsg(*laserCloud2, *laserCloud);
    // 连续两帧点云积分
    pcl::PointCloud<pcl::PointXYZI> laserCloud_integral;
    laserCloud_integral = *laserCloud + *laserCloud_last;
    // 更新上一帧的点云
    *laserCloud_last = *laserCloud;
    // 将积分后的点云赋值给laserCloud
    *laserCloud = laserCloud_integral;

    // 正常处理
    pcl::PointXYZI point;
    laserCloudCrop->clear(); // 解放变量
    int laserCloudSize = laserCloud->points.size();
    for (int i = 0; i < laserCloudSize; i++)
    {
        point = laserCloud->points[i];
        // ROS_WARN("point.intensity:%f:", point.intensity);
        float pointX = point.x;
        float pointY = point.y;
        float pointZ = point.z;
        // 计算点云到车体的的距离
        float dis = sqrt((pointX - vehicleX) * (pointX - vehicleX) +
                         (pointY - vehicleY) * (pointY - vehicleY));
        // 主要就是确认是否放入laserCloudCrop（确认为裁减过的有效的）
        // 此局部地形分析的范围在5m以内
        if (pointZ - vehicleZ > minRelZ - disRatioZ * dis && // disRatioZ为点云处理的高度与距离的比例， 点云越近点云就得越高,当距离dis=0时点云相对车体高恰好在min/maxRelZ之间,点云越远这个约束越无效
            pointZ - vehicleZ < maxRelZ + disRatioZ * dis &&
            dis < terrainVoxelSize * (terrainVoxelHalfWidth + 1) && // terrainVoxelSize的体素网格的边长*半边长，说明以车子为中心，距离不能超过体素网格区域内，这个方是超出网格范围的最短方向
            dis < terrainMapRadius)                                 // 人为设置一个小于等于10.0的分析半径
        {
            point.x = pointX;
            point.y = pointY;
            point.z = pointZ;
            point.intensity = laserCloudTime - systemInitTime; // 以时间衰减作为点云强度
            // laserCloudCrop代表筛选过的有效的
            laserCloudCrop->push_back(point);
        }
    }
    newlaserCloud = true;
}

// cloud clearing callback function
void clearingHandler(const std_msgs::Float32::ConstPtr &dis)
{
    noDataInited = 0;
    clearingDis = dis->data;
    clearingCloud = true;
}

/*
    模块封装(ldq 2023.12.9)
*/
pcl::PointXYZI point;   // point暂存值
int laserCloudCropSize; // 雷达一帧下来初始点云数量

void roll_over_terrainVoxel();                    // 回滚地形中心和车体位置重合
void laserCloudCrop_to_terrainVoxelCloud();       // 初始点云压入对应地形体素位置
void terrainVoxelCloud_to_terrainVoxelCloudPtr(); // 地形点云下采样和条件筛选更新
void terrainVoxelCloud_to_terrainCloud();         // 将地形中心周围5格范围内的10*10的区域地形点云压入terrainCloud
void init_planarVoxel_all();
void terrainCloud_to_planarPointElev(int indX, int indY); // 限制在高程上下限中的高度会影响周围3*3区域,z高度压入Elev中
void terrainCloud_to_planarVoxelDyObs(int indX, int indY);
void laserCloudCrop_to_planarVoxelDyObs();
void planarPointElev_to_planarVoxelElev();
void terrainCloud_to_terrainCloudELev(int terrainCloudSize); // 确认最后的有效墙面障碍点云
void planarVoxelNum_to_terrainCloudELev();

int main(int argc, char **argv)
{
    ros::init(argc, argv, "terrainAnalysis");
    ros::NodeHandle nh;
    ros::NodeHandle nhPrivate = ros::NodeHandle("~");

    nhPrivate.getParam("scanVoxelSize", scanVoxelSize);
    nhPrivate.getParam("decayTime", decayTime);
    nhPrivate.getParam("noDecayDis", noDecayDis);
    nhPrivate.getParam("clearingDis", clearingDis);
    nhPrivate.getParam("useSorting", useSorting);
    nhPrivate.getParam("quantileZ", quantileZ);
    nhPrivate.getParam("considerDrop", considerDrop);
    nhPrivate.getParam("limitGroundLift", limitGroundLift);
    nhPrivate.getParam("maxGroundLift", maxGroundLift);
    nhPrivate.getParam("clearDyObs", clearDyObs);
    nhPrivate.getParam("minDyObsDis", minDyObsDis);
    nhPrivate.getParam("minDyObsAngle", minDyObsAngle);
    nhPrivate.getParam("minDyObsRelZ", minDyObsRelZ);
    nhPrivate.getParam("minDyObsVFOV", minDyObsVFOV);
    nhPrivate.getParam("maxDyObsVFOV", maxDyObsVFOV);
    nhPrivate.getParam("minDyObsPointNum", minDyObsPointNum);
    nhPrivate.getParam("noDataObstacle", noDataObstacle);
    nhPrivate.getParam("noDataBlockSkipNum", noDataBlockSkipNum);
    nhPrivate.getParam("minBlockPointNum", minBlockPointNum);
    nhPrivate.getParam("vehicleHeight", vehicleHeight);
    nhPrivate.getParam("voxelPointUpdateThre", voxelPointUpdateThre);
    nhPrivate.getParam("voxelTimeUpdateThre", voxelTimeUpdateThre);
    nhPrivate.getParam("minRelZ", minRelZ);
    nhPrivate.getParam("maxRelZ", maxRelZ);
    nhPrivate.getParam("disRatioZ", disRatioZ);
    nhPrivate.getParam("terrainMapRadius", terrainMapRadius);
    // nhPrivate.param("planarVoxelSize", planarVoxelSize, 0.2);

    ros::Subscriber subOdometry = nh.subscribe<nav_msgs::Odometry>("/odom_interface", 5, odometryHandler);

    ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/cloud_interface", 5, laserCloudHandler);

    ros::Subscriber subClearing = nh.subscribe<std_msgs::Float32>("/map_clearing", 5, clearingHandler);

    ros::Publisher pubLaserCloud = nh.advertise<sensor_msgs::PointCloud2>("/terrain_map", 5);

    for (int i = 0; i < terrainVoxelNum; i++)
    {
        // 长期维护的地形体素地图，不会直接刷新，而是缓存
        terrainVoxelCloud[i].reset(new pcl::PointCloud<pcl::PointXYZI>());
    }

    downSizeFilter.setLeafSize(scanVoxelSize, scanVoxelSize, scanVoxelSize);

    ros::Rate rate(20);
    bool status = ros::ok();
    while (status)
    {
        ros::spinOnce(); // laserCloud->clear(),得到laserCloudCrop[]

        if (newlaserCloud) // 做完点云处理再进来
        {
            newlaserCloud = false;
            //** terrainVoxelCloud[]滚动位置
            roll_over_terrainVoxel(); // 若某次车体位置在更新之后与地形点云中心不重合,点云滚动，将地形中心设置为车体位置
            //** laserCloudCrop--->terrainVoxelCloud, terrainVoxelCloud没有clear()！！！
            // 下面这一段代码主要是先将点云坐标单位转为体素网格数为单位，将点云压入对应体素网格位置，并记录对应网格中的新增点云数量
            laserCloudCrop_to_terrainVoxelCloud();
            // 遍历terrainVoxelCloud中所有的体素网格，分别都进行一次下采样
            terrainVoxelCloud_to_terrainVoxelCloudPtr();

            // terrainVoxelCloud[]--->terrainCloud[]
            terrainCloud->clear();
            terrainVoxelCloud_to_terrainCloud();

            // 先初始化所有的平面体素数组
            init_planarVoxel_all();

            int terrainCloudSize = terrainCloud->points.size(); // 中心区域内有多少点云
            /**
             * @brief 遍历所有的点云，把点云z值放入高度体素网格对应位置，同时还要以3*3的区域内都使用这个高度z
             *1. 一个体素位置可能有多个高程z，后续将筛选出最小或较小值作为这个位置的地面高度
             *2. 区域3*3内有0.6m，可以尝试改小这个范围，比如中心周围5点区域或只有中心点等，来兼容坡面
             */
            for (int i = 0; i < terrainCloudSize; i++)
            {
                point = terrainCloud->points[i];
                int indX = int((point.x - vehicleX + planarVoxelSize / 2) / planarVoxelSize) + planarVoxelHalfWidth;
                int indY = int((point.y - vehicleY + planarVoxelSize / 2) / planarVoxelSize) + planarVoxelHalfWidth;
                // 同样先将点云坐标单位转成平面体素个数为单位,定位到点云对应平面体素位置,
                // 再提取出该点云的有效高度,若高度有效(在[minRelZ,maxRelZ]),这个高度会影响周围3*3区域的平面高程
                terrainCloud_to_planarPointElev(indX, indY);

                // clearDyObs为true，即对这个地形点云作动态障碍物判定处理
                terrainCloud_to_planarVoxelDyObs(indX, indY);
            }

            // 这时已经脱离了地形点云框架
            // todo 这一段跟上一段有什么关系
            // 回到最原始的刚从订阅回调函数出来的laserCloudCrop,只要存在点云相对车体在世界坐标系下的垂直倾角足够大,马上又抹调掉该体素下所有动态障碍物标记
            laserCloudCrop_to_planarVoxelDyObs();

            planarPointElev_to_planarVoxelElev(); // 从小到大保留高程代表,或取最小

            //**核心——最终的地形信息,三个条件从terrainCloud中筛选有效墙面送入terrainCloudElev，即为发布的地形点云
            // terrainCloudElev->clear(), terrainCloud[]--->terrainCloudElev[]
            terrainCloud_to_terrainCloudELev(terrainCloudSize);

            /**
             * @brief 这里不重要noDataObstacl = false,根本就不跑
             * 1. 在没有数据的区域进行衰减处理，通过计算平面体素的边界值，判断是否属于边界。
             * 2. 如果是边界，根据边界的位置，构造出地面区域的轮廓点，然后加入到 terrainCloudElev 中
             */
            planarVoxelNum_to_terrainCloudELev();

            clearingCloud = false; // 清理完一轮就可以了

            // publish points with elevation
            sensor_msgs::PointCloud2 terrainCloud2;
            pcl::toROSMsg(*terrainCloudElev, terrainCloud2);
            terrainCloud2.header.stamp = ros::Time().fromSec(laserCloudTime);
            terrainCloud2.header.frame_id = "map";
            pubLaserCloud.publish(terrainCloud2);
        }
        status = ros::ok();
        rate.sleep();
    }
    return 0;
}

void roll_over_terrainVoxel()
{
    /*terrain voxel roll over
        terrainVoxelShiftX，terrainVoxelShiftX为地形中心在世界坐标系下的偏移量（以体素个数为单位），初始地形中心设置在世界原点
        terrainVoxelSize为地形体素的边长
        terrainVoxelCenX和terrainVoxelCenY是地形体素的中心XY坐标
    */
    float terrainVoxelCenX = terrainVoxelSize * terrainVoxelShiftX;
    float terrainVoxelCenY = terrainVoxelSize * terrainVoxelShiftY;
    // 下列这一段代码的作用是若某次车体位置在更新之后与地形点云中心不重合,点云滚动，将地形中心设置为车体位置
    while (vehicleX - terrainVoxelCenX < -terrainVoxelSize)
    {
        for (int indY = 0; indY < terrainVoxelWidth; indY++)
        {
            // terrainVoxelCloudPtr时x_21（indY），即最后一排的原来的y位置的点云
            pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloudPtr =
                terrainVoxelCloud[terrainVoxelWidth * (terrainVoxelWidth - 1) +
                                  indY];
            for (int indX = terrainVoxelWidth - 1; indX >= 1; indX--)
            {
                // 点云整体向后平移一个体素栅格，后一个的索引位置置为前一个索引位置的点云信息，是指针指向的修改
                /*
                    数据结构，将矩阵拉值了为如下示意。
                    x1_(y1 y2 y3...) | x2_(y1 y2 y3...) |
                */
                terrainVoxelCloud[terrainVoxelWidth * indX + indY] =
                    terrainVoxelCloud[terrainVoxelWidth * (indX - 1) + indY];
            }
            // 显然后一个拿到前一个之后，因为indX>=1,即indX=0 处的点云是没得拿要删去
            // 但是原来第一排已经被第二排指向了，即将第一排指向原来最后一派再删去
            terrainVoxelCloud[indY] = terrainVoxelCloudPtr;
            terrainVoxelCloud[indY]->clear();
        }
        // 地形和车体偏移量向后移
        terrainVoxelShiftX--;
        terrainVoxelCenX = terrainVoxelSize * terrainVoxelShiftX;
    }

    while (vehicleX - terrainVoxelCenX > terrainVoxelSize)
    {
        for (int indY = 0; indY < terrainVoxelWidth; indY++)
        {
            pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloudPtr =
                terrainVoxelCloud[indY];
            for (int indX = 0; indX < terrainVoxelWidth - 1; indX++)
            {
                terrainVoxelCloud[terrainVoxelWidth * indX + indY] =
                    terrainVoxelCloud[terrainVoxelWidth * (indX + 1) + indY];
            }
            terrainVoxelCloud[terrainVoxelWidth * (terrainVoxelWidth - 1) +
                              indY] = terrainVoxelCloudPtr;
            terrainVoxelCloud[terrainVoxelWidth * (terrainVoxelWidth - 1) + indY]
                ->clear();
        }
        terrainVoxelShiftX++;
        terrainVoxelCenX = terrainVoxelSize * terrainVoxelShiftX;
    }

    while (vehicleY - terrainVoxelCenY < -terrainVoxelSize)
    {
        for (int indX = 0; indX < terrainVoxelWidth; indX++)
        {
            pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloudPtr =
                terrainVoxelCloud[terrainVoxelWidth * indX +
                                  (terrainVoxelWidth - 1)];
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
            pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloudPtr =
                terrainVoxelCloud[terrainVoxelWidth * indX];
            for (int indY = 0; indY < terrainVoxelWidth - 1; indY++)
            {
                terrainVoxelCloud[terrainVoxelWidth * indX + indY] =
                    terrainVoxelCloud[terrainVoxelWidth * indX + (indY + 1)];
            }
            terrainVoxelCloud[terrainVoxelWidth * indX +
                              (terrainVoxelWidth - 1)] = terrainVoxelCloudPtr;
            terrainVoxelCloud[terrainVoxelWidth * indX + (terrainVoxelWidth - 1)]->clear();
        }
        terrainVoxelShiftY++;
        terrainVoxelCenY = terrainVoxelSize * terrainVoxelShiftY;
    }
}

void laserCloudCrop_to_terrainVoxelCloud()
{
    laserCloudCropSize = laserCloudCrop->points.size();
    for (int i = 0; i < laserCloudCropSize; i++)
    {
        point = laserCloudCrop->points[i];

        // 将点云的世界坐标（m为单位）转为体素网格数量为单位，前面四个循环已经将车体调到体素网格正中心
        int indX = int((point.x - vehicleX + terrainVoxelSize / 2) /
                       terrainVoxelSize) +
                   terrainVoxelHalfWidth;
        int indY = int((point.y - vehicleY + terrainVoxelSize / 2) /
                       terrainVoxelSize) +
                   terrainVoxelHalfWidth;
        // 当点云在左下角靠原点一侧时(即下图a点位置)，比如-1.2向上取整为-1，但其实要取的是-2，所以下面作的是这个处理
        /*
                |
                |      车
                |   a
                O_______________
        */
        if (point.x - vehicleX + terrainVoxelSize / 2 < 0)
            indX--;
        if (point.y - vehicleY + terrainVoxelSize / 2 < 0)
            indY--;
        // 防止计算出的坐标越界
        if (indX >= 0 && indX < terrainVoxelWidth && indY >= 0 &&
            indY < terrainVoxelWidth)
        {
            // 可以看做用一维数组表示二维数组
            terrainVoxelCloud[terrainVoxelWidth * indX + indY]->push_back(point); // 将点云压入对应体素网格位置
            // terrainVoxelUpdateNum记录每一个体素网格中存入点云的数量
            terrainVoxelUpdateNum[terrainVoxelWidth * indX + indY]++; // 记录对应网格中的新增点云数量
        }
    }
}

void terrainVoxelCloud_to_terrainVoxelCloudPtr()
{
    for (int ind = 0; ind < terrainVoxelNum; ind++)
    {
        if (terrainVoxelUpdateNum[ind] >= voxelPointUpdateThre ||                                   // 这个网格新增太多点云了
            laserCloudTime - systemInitTime - terrainVoxelUpdateTime[ind] >= voxelTimeUpdateThre || // 这个网格太久没更新了
            clearingCloud)                                                                          // 刚刚清理完整张点云地形图(重置地形图)
        {
            // 体素栅格中第ind块点云,一整块进行下采样,下采样后保留为部分点云代表,节省资源
            pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloudPtr = terrainVoxelCloud[ind]; // 这是一个指针，操作的还是原来的terrainVoxelCloud
            // 对laserCloudDwz进行下采样,存入laserCloudDwz
            laserCloudDwz->clear();
            downSizeFilter.setInputCloud(terrainVoxelCloudPtr);
            downSizeFilter.filter(*laserCloudDwz);

            terrainVoxelCloudPtr->clear(); // 如果置了clearingDis,下面的push不可能跑了,等价于清除了(只是后面循环白跑了)
            int laserCloudDwzSize = laserCloudDwz->points.size();
            // 遍历这一块点云
            for (int i = 0; i < laserCloudDwzSize; i++)
            {
                point = laserCloudDwz->points[i];
                // 计算所有点到车体的距离
                float dis = sqrt((point.x - vehicleX) * (point.x - vehicleX) +
                                 (point.y - vehicleY) * (point.y - vehicleY));
                // 满足下列三个条件的点云才会被保存
                if (point.z - vehicleZ > minRelZ - disRatioZ * dis &&    // 1. 点云越近点云就得越高,当距离dis=0时点云相对车体高恰好在min/maxRelZ之间,点云越远这个约束越无效
                    point.z - vehicleZ < maxRelZ + disRatioZ * dis &&    //
                    (laserCloudTime - systemInitTime - point.intensity < // 2. 此时此刻的点云和第一帧点云的时间差小于衰减时间
                         decayTime ||                                    //         /
                     dis < noDecayDis) &&                                // 或者和车体点云距离在无时间衰减范围内
                    !(dis < clearingDis && clearingCloud))               // 3. 距离小于清除距离且开启了清除地形点云图标志位
                {
                    terrainVoxelCloudPtr->push_back(point);
                }
            }
            // 表示当前网格所有点云已被更新
            terrainVoxelUpdateNum[ind] = 0;
            terrainVoxelUpdateTime[ind] = laserCloudTime - systemInitTime;
        }
    }
}

void terrainVoxelCloud_to_terrainCloud()
{
    // 全部体素都要
    for (int indX = terrainVoxelHalfWidth - 10;
         indX <= terrainVoxelHalfWidth + 10; indX++)
    {
        // 遍历Y轴上所有的体素栅格
        for (int indY = terrainVoxelHalfWidth - 8;
             indY <= terrainVoxelHalfWidth + 8; indY++)
        {
            // 将地形点云传入terrainCloud
            *terrainCloud += *terrainVoxelCloud[terrainVoxelWidth * indX + indY];
        }
    }
}

void init_planarVoxel_all()
{
    for (int i = 0; i < planarVoxelNum; i++)
    {
        planarVoxelElev[i] = 0;
        planarVoxelEdge[i] = 0;
        planarVoxelDyObs[i] = 0;
        planarPointElev[i].clear();
    }
}
void terrainCloud_to_planarPointElev(int indX, int indY)
{
    if (point.x - vehicleX + planarVoxelSize / 2 < 0)
        indX--;
    if (point.y - vehicleY + planarVoxelSize / 2 < 0)
        indY--;
    // 在体素栅格中，需要被进行地面分割的点云满足以下要求，才会被放入
    if (point.z - vehicleZ > minRelZ && point.z - vehicleZ < maxRelZ) // 点云到里程计高度在[minRelZ,maxRelZ]
    {
        for (int dX = -1; dX <= 1; dX++) // 以点云在平面体素中的位置为中心,周围3*3的体素位置都插入该点云的z高度,即该点云会影响周围3*3区域的地面高度
        {
            for (int dY = -1; dY <= 1; dY++)
            {
                if (indX + dX >= 0 && indX + dX < planarVoxelWidth && // 防止超出平面体素区域范围
                    indY + dY >= 0 && indY + dY < planarVoxelWidth)
                {
                    planarPointElev[planarVoxelWidth * (indX + dX) + indY + dY]
                        .push_back(point.z); // 以pushback形式插入,一个平面体素位置会有多个高度
                }
            }
        }
    }
}
void terrainCloud_to_planarVoxelDyObs(int indX, int indY)
{
    if (clearDyObs)
    {
        // 防止索引越界
        if (indX >= 0 && indX < planarVoxelWidth &&
            indY >= 0 && indY < planarVoxelWidth)
        {
            float pointX1 = point.x - vehicleX;
            float pointY1 = point.y - vehicleY;
            float pointZ1 = point.z - vehicleZ;
            // 计算点云到车体的距离
            float dis1 = sqrt(pointX1 * pointX1 + pointY1 * pointY1);
            // 只对距离车体大于minDyObsDis的点云做处理,车体的位置由/Odometry状态估计订阅回调决定(lio算法发布)
            if (dis1 > minDyObsDis)
            {
                // 点云和车体连线和水平面的夹角
                float angle1 = atan2(pointZ1 - minDyObsRelZ, dis1) * 180.0 / PI;
                // 点云和车体连线和水平面的夹角大于 minDyObsAngle,即点云相对车体在世界坐标系下的垂直倾角足够大
                if (angle1 > minDyObsAngle)
                {
                    // point234代表点云世界坐标系分别做绕Z,y,x轴的旋转
                    // point2以z为旋转轴,转到车体方向
                    float pointX2 =
                        pointX1 * cosVehicleYaw + pointY1 * sinVehicleYaw;
                    float pointY2 =
                        -pointX1 * sinVehicleYaw + pointY1 * cosVehicleYaw;
                    float pointZ2 = pointZ1;
                    // 再以y为旋转轴
                    float pointX3 =
                        pointX2 * cosVehiclePitch - pointZ2 * sinVehiclePitch;
                    float pointY3 = pointY2;
                    float pointZ3 = pointX2 * sinVehiclePitch + pointZ2 * cosVehiclePitch;
                    // 最后以x为旋转轴,此时完成了世界坐标系转移到整车坐标系
                    float pointX4 = pointX3;
                    float pointY4 = pointY3 * cosVehicleRoll + pointZ3 * sinVehicleRoll;
                    float pointZ4 =
                        -pointY3 * sinVehicleRoll + pointZ3 * cosVehicleRoll;

                    // 转为车体看到的点云位置
                    float dis4 = sqrt(pointX4 * pointX4 + pointY4 * pointY4);
                    float angle4 = atan2(pointZ4, dis4) * 180.0 / PI;
                    if (angle4 > minDyObsVFOV && angle4 < maxDyObsVFOV) // minDyObsVFOV和maxDyObsVFOV为视场角范围内
                    {
                        planarVoxelDyObs[planarVoxelWidth * indX + indY]++; // 转车体系后还能看说明动态点云有效,在对应体素位置记录一个点云有效点云信息
                    }
                }
            }
            else // 存在点云距离比较近,不作动态障碍物判断,直接认为是动态的障碍物,赋给这个体素位置判定标准的最小动态障碍物点云数量
            {
                planarVoxelDyObs[planarVoxelWidth * indX + indY] += minDyObsPointNum;
            }
        }
    }
}
void laserCloudCrop_to_planarVoxelDyObs()
{
    if (clearDyObs)
    {
        for (int i = 0; i < laserCloudCropSize; i++)
        {
            point = laserCloudCrop->points[i];
            // 坐标转为平面体素数量为单位
            int indX = int((point.x - vehicleX + planarVoxelSize / 2) / planarVoxelSize) + planarVoxelHalfWidth;
            int indY = int((point.y - vehicleY + planarVoxelSize / 2) / planarVoxelSize) + planarVoxelHalfWidth;

            if (point.x - vehicleX + planarVoxelSize / 2 < 0)
                indX--;
            if (point.y - vehicleY + planarVoxelSize / 2 < 0)
                indY--;
            if (indX >= 0 && indX < planarVoxelWidth && indY >= 0 &&
                indY < planarVoxelWidth)
            {
                float pointX1 = point.x - vehicleX;
                float pointY1 = point.y - vehicleY;
                float pointZ1 = point.z - vehicleZ;

                float dis1 = sqrt(pointX1 * pointX1 + pointY1 * pointY1);
                float angle1 = atan2(pointZ1 - minDyObsRelZ, dis1) * 180.0 / PI;
                if (angle1 > minDyObsAngle)
                {
                    planarVoxelDyObs[planarVoxelWidth * indX + indY] = 0;
                }
            }
        }
    }
}

void planarPointElev_to_planarVoxelElev()
{
    if (useSorting) // 从小到大保留高程代表
    {
        for (int i = 0; i < planarVoxelNum; i++)
        {
            // 地面高程点云的个数,planarPointElev里面装的是每一个点云的高度信息
            int planarPointElevSize = planarPointElev[i].size();
            if (planarPointElevSize > 0)
            {
                // 对平面点高程进行排序
                sort(planarPointElev[i].begin(), planarPointElev[i].end());
                // 考虑地面附近高程最小值会改变
                int quantileID = int(quantileZ * planarPointElevSize);
                // 防止地面高程点云的个数越界
                if (quantileID < 0)
                    quantileID = 0;
                else if (quantileID >= planarPointElevSize)
                    quantileID = planarPointElevSize - 1;
                if (planarPointElev[i][quantileID] >
                        planarPointElev[i][0] + maxGroundLift && // 如果要限制地面高度且这个存活下来的高程代表在阈值以外
                    limitGroundLift)
                {
                    planarVoxelElev[i] = planarPointElev[i][0] + maxGroundLift; // 直接赋予作为限制的最大地面高度
                }
                else
                {
                    planarVoxelElev[i] = planarPointElev[i][quantileID]; // 不用限制地面高度的话,直接赋值给他
                }
            }
        }
    }
    else // 只要最小的高程信息,其他全部舍弃
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
}

void terrainCloud_to_terrainCloudELev(int terrainCloudSize)
{
    terrainCloudElev->clear();
    int terrainCloudElevSize = 0;
    // terrainCloudSize为地形体素栅格的个数
    for (int i = 0; i < terrainCloudSize; i++)
    {
        point = terrainCloud->points[i];
        // 保留满足以下条件的点云
        // 点云到车体的高度差在[minRelZ, maxRelZ]中
        if (point.z - vehicleZ > minRelZ && point.z - vehicleZ < maxRelZ)
        {
            // 将点云世界坐标单位转成平面体素个数单位(转成体素索引)
            int indX = int((point.x - vehicleX + planarVoxelSize / 2) /
                           planarVoxelSize) +
                       planarVoxelHalfWidth;
            int indY = int((point.y - vehicleY + planarVoxelSize / 2) /
                           planarVoxelSize) +
                       planarVoxelHalfWidth;

            if (point.x - vehicleX + planarVoxelSize / 2 < 0)
                indX--;
            if (point.y - vehicleY + planarVoxelSize / 2 < 0)
                indY--;
            // 防止越界
            if (indX >= 0 && indX < planarVoxelWidth && indY >= 0 &&
                indY < planarVoxelWidth)
            {
                if (planarVoxelDyObs[planarVoxelWidth * indX + indY] <
                        minDyObsPointNum || // 不作动态障碍物分析或不认为是动态障碍物
                    !clearDyObs)
                {
                    // planarVoxelElev和planarPointElev的区别是什么-----planarVoxelElev为planarPointElev中代表性的一个高度
                    float disZ = point.z - planarVoxelElev[planarVoxelWidth * indX + indY]; // disZ为该地形点相对于对应体素分析后高程的高度差
                    if (considerDrop)
                        disZ = fabs(disZ); // 浮点数绝对值
                    int planarPointElevSize = planarPointElev[planarVoxelWidth * indX + indY].size();
                    /*三个条件
                      1. 点云高度不能和地面(地面由planarVoxelElev筛选出的最低点决定)齐平
                      2. 点云墙面障碍要在车体高度内,如果障碍物高于车体,说明是一个洞,可以穿过去
                      3. 这个体素位置扫到的有效高度高程要大于堵塞阈值要求的数量
                    */
                    if (disZ >= 0 && disZ < vehicleHeight &&
                        planarPointElevSize >= minBlockPointNum)
                    {
                        terrainCloudElev->push_back(point);                              // 最终被认为是有效遮挡的墙面的点云
                        terrainCloudElev->points[terrainCloudElevSize].intensity = disZ; // 遮挡强度,非常低小台阶的话说不定还能撞上去
                        terrainCloudElevSize++;
                    }
                }
            }
        }
    }
}

void planarVoxelNum_to_terrainCloudELev()
{
    if (noDataObstacle && noDataInited == 2)
    {
        for (int i = 0; i < planarVoxelNum; i++)
        {
            int planarPointElevSize = planarPointElev[i].size();
            if (planarPointElevSize < minBlockPointNum) // 平面体素中的高程点数小于 minBlockPointNum
            {
                planarVoxelEdge[i] = 1; // 标记为边界
            }
        }

        for (int noDataBlockSkipCount = 0;
             noDataBlockSkipCount < noDataBlockSkipNum; // noDataBlockSkipNum=0, 无效循环,不知道在干嘛
             noDataBlockSkipCount++)
        {
            for (int i = 0; i < planarVoxelNum; i++) // 遍历所有
            {
                if (planarVoxelEdge[i] >= 1) // 对每个边界体素进行处理
                {
                    int indX = int(i / planarVoxelWidth);
                    int indY = i % planarVoxelWidth;
                    bool edgeVoxel = false;
                    for (int dX = -1; dX <= 1; dX++)
                    {
                        for (int dY = -1; dY <= 1; dY++)
                        {
                            if (indX + dX >= 0 && indX + dX < planarVoxelWidth &&
                                indY + dY >= 0 && indY + dY < planarVoxelWidth)
                            {
                                if (planarVoxelEdge[planarVoxelWidth * (indX + dX) + indY +
                                                    dY] < planarVoxelEdge[i])
                                {
                                    edgeVoxel = true; // 判断其周围的相邻体素都得是边缘体素，才确认真的是
                                }
                            }
                        }
                    }

                    if (!edgeVoxel) // 对于每个边界体素
                        planarVoxelEdge[i]++;
                }
            }
        }

        for (int i = 0; i < planarVoxelNum; i++) // 遍历所有
        {
            if (planarVoxelEdge[i] > noDataBlockSkipNum) // 确认为有效边界体素
            {
                int indX = int(i / planarVoxelWidth); // 行号除法
                int indY = i % planarVoxelWidth;      // 列号取余

                // 恢复成世界坐标
                point.x = planarVoxelSize * (indX - planarVoxelHalfWidth) + vehicleX;
                point.y = planarVoxelSize * (indY - planarVoxelHalfWidth) + vehicleY;
                point.z = vehicleZ;
                point.intensity = vehicleHeight;

                // 在单个体素planarVoxelSize范围内周围构造附加点添加到最终 terrainCloudElev 中
                point.x -= planarVoxelSize / 4.0;
                point.y -= planarVoxelSize / 4.0;
                terrainCloudElev->push_back(point);

                point.x += planarVoxelSize / 2.0;
                terrainCloudElev->push_back(point);

                point.y += planarVoxelSize / 2.0;
                terrainCloudElev->push_back(point);

                point.x -= planarVoxelSize / 2.0;
                terrainCloudElev->push_back(point);
            }
        }
    }
}