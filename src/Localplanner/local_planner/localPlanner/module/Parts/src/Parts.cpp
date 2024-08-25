#include "Parts.h"

#define PLOTPATHSET 1 // 开启可行路径可视化
const double PI = 3.1416;
string pathFolder;
double terrainVoxelSize = 0.2;
bool checkRotObstacle = false;
const int laserCloudStackNum = 1;
int laserCloudCount = 0;
double maxSpeed = 1.0;
float detourWei = 1.0;

double goalX = 0;
double goalY = 0;

float joySpeed = 0; // 车体速度当前值
double joyDir = 0;  // joyDir为车体到目标点之间的yaw角

const int pathNum = 343; // 一个10度方向待规划的路径数
const int groupNum = 7;
float gridVoxelSize = 0.02;
float searchRadius = 0.45;
float gridVoxelOffsetX = 3.2;
float gridVoxelOffsetY = 4.5;
const int gridVoxelNumX = 161;
const int gridVoxelNumY = 451;
const int gridVoxelNum = 161 * 451;

pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloud(new pcl::PointCloud<pcl::PointXYZI>());     //!  存储激光雷达扫描数据
pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCrop(new pcl::PointCloud<pcl::PointXYZI>()); //! 存储经过裁剪的激光雷达扫描数据
pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudDwz(new pcl::PointCloud<pcl::PointXYZI>());  //! 存储经过某种处理的激光雷达扫描数据

pcl::PointCloud<pcl::PointXYZI>::Ptr terrainCloud(new pcl::PointCloud<pcl::PointXYZI>()); // 存储地形相关的点云数据
pcl::PointCloud<pcl::PointXYZI>::Ptr terrainCloudCrop(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr terrainCloudDwz(new pcl::PointCloud<pcl::PointXYZI>());

pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudStack[1];                                  // 包含 laserCloudStackNum 个指向类型为 pcl::PointXYZI 的点云对象的指针。
pcl::PointCloud<pcl::PointXYZI>::Ptr plannerCloud(new pcl::PointCloud<pcl::PointXYZI>()); // 可能用于存储路径规划相关的点云数据
pcl::PointCloud<pcl::PointXYZI>::Ptr plannerCloudCrop(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr PannerAtuCloud(new pcl::PointCloud<pcl::PointXYZI>());

pcl::PointCloud<pcl::PointXYZI>::Ptr addedObstacles(new pcl::PointCloud<pcl::PointXYZI>()); // 存储被添加的障碍物的数据
pcl::PointCloud<pcl::PointXYZI>::Ptr addedObstaclesCrop(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZ>::Ptr startPaths[7]; // 存储不同路径组的起始路径或轨迹数据

#if PLOTPATHSET == 1
pcl::PointCloud<pcl::PointXYZI>::Ptr paths[343];                                       // 用于存储不同路径的数据
pcl::PointCloud<pcl::PointXYZI>::Ptr freePaths(new pcl::PointCloud<pcl::PointXYZI>()); // 存储所有可行路径的数据
#endif

int pathList[343] = {0};
float endDirPathList[343] = {0};             // 343条路径分成7组，每组49条，这里记录每一条路径的终点相对车头方向
int clearPathList[36 * 343] = {0};           // 可能用于存储路径的分辨率信息
float pathPenaltyList[36 * 343] = {0};       // 可能用于存储路径的惩罚值
float clearPathPerGroupScore[36 * 7] = {0};  // 能用于存储每个路径组的分辨率分数
std::vector<int> correspondences[161 * 451]; // 用于网格和路径相对应

bool newTerrainCloud = false; // 是否有新的地形云数据可用

// 初始化时间戳
double odomTime = 0;
// 初始化车体的位姿
float vehicleRoll = 0, vehiclePitch = 0, vehicleYaw = 0;
float vehicleX = 0, vehicleY = 0, vehicleZ = 0;
pcl::VoxelGrid<pcl::PointXYZI> laserDwzFilter, terrainDwzFilter; // 对激光雷达数据和地形数据进行体素化滤波，后的点云数据存储

pcl::PointXYZI point; // 点云暂存值
int plannerCloudSize; // 待处理新增点云数量

float sinVehicleYaw;
float cosVehicleYaw;

bool pathFound = false;       // 找到最终规划出来的路径
double pathRange;             // 路径范围(黄色)
float relativeGoalDis;        // 目标点相对车体的距离
float relativeGoalDis_global; // 全局点相对车体的距离
double defPathScale;          // 路径规模初始值
float rotAng;                 // 相对joyDir目标点方向的相对方向角度（-180，180）
float angDiff;                // rotAng叠加上joyDir
float minObsAngCW;            // minObsAngCW是考虑车体的直径和考虑侧方障碍物的情况下，车顺时针能旋转的最小角度
float minObsAngCCW;           // minObsAngCCW是是考虑车体的直径和考虑侧方障碍物的情况下，车逆时针能旋转的最小角度
float angOffset;              // angOffset为长方形车体的对角线方向(最容易碰撞，用于侧向碰撞检测)
float diameter;               // diameter为车体的半对角线长度
int plannerCloudCropSize;     // plannerCloudCropSize为待处理的点云数量
std_msgs::Float32 slowDown;
float planRangeK;

/*
 * 读取ply文件，获取点云的采样个数
 */
int readPlyHeader(FILE *filePtr)
{
    char str[50];
    int val, pointNum;
    string strCur, strLast;
    while (strCur != "end_header")
    {
        // 读取filePtr文件的字符，读取不成功则返回-1
        val = fscanf(filePtr, "%s", str);
        if (val != 1)
        {
            printf("\nError reading input files, exit.\n\n");
            exit(1);
        }

        strLast = strCur;
        strCur = string(str);
        // ply文件中有这样一行 element vertex 10727
        // 其中10727的含义就是这个ply文件中含有的点云采样个数
        if (strCur == "vertex" && strLast == "element")
        {
            val = fscanf(filePtr, "%d", &pointNum);
            if (val != 1)
            {
                printf("\nError reading input files, exit.\n\n");
                exit(1);
            }
        }
    }

    return pointNum;
}

/*
 * 读取startPaths.ply文件，并将点云信息传入到startPaths中
 * startPaths的意义是第一次采样的路径点，可以理解成
 */
void readStartPaths()
{
    string fileName = pathFolder + "/startPaths.ply";

    FILE *filePtr = fopen(fileName.c_str(), "r");
    if (filePtr == NULL)
    {
        printf("\nCannot read input files, exit.\n\n");
        exit(1);
    }

    int pointNum = readPlyHeader(filePtr); // 专门获取点云数量

    pcl::PointXYZ point;
    int val1, val2, val3, val4, groupID;
    for (int i = 0; i < pointNum; i++)
    {
        // val是成功标志位
        val1 = fscanf(filePtr, "%f", &point.x);
        val2 = fscanf(filePtr, "%f", &point.y);
        val3 = fscanf(filePtr, "%f", &point.z);
        val4 = fscanf(filePtr, "%d", &groupID);

        if (val1 != 1 || val2 != 1 || val3 != 1 || val4 != 1)
        {
            printf("\nError reading input files, exit.\n\n");
            exit(1);
        }

        if (groupID >= 0 && groupID < groupNum)
        {
            startPaths[groupID]->push_back(point);
        }
    }
    // 关闭文件
    fclose(filePtr);
}

// 编译选项，开启路径可视化时进行处理
#if PLOTPATHSET == 1
/*
 * 读取paths.ply文件，把点云信息存入到paths中
 * paths的意义是路径点的集合，此函数就是在生成初始化的黄色路径点集合
 */
void readPaths()
{
    string fileName = pathFolder + "/paths.ply";

    FILE *filePtr = fopen(fileName.c_str(), "r");
    if (filePtr == NULL)
    {
        printf("\nCannot read input files, exit.\n\n");
        exit(1);
    }
    // 获取该文件中点云的采样个数
    int pointNum = readPlyHeader(filePtr);

    pcl::PointXYZI point;
    int pointSkipNum = 30;
    int pointSkipCount = 0;
    int val1, val2, val3, val4, val5, pathID;
    // 读取点云的X,Y,Z,intensity和pathID
    for (int i = 0; i < pointNum; i++)
    {
        val1 = fscanf(filePtr, "%f", &point.x);
        val2 = fscanf(filePtr, "%f", &point.y);
        val3 = fscanf(filePtr, "%f", &point.z);
        // pathID的意义是paths路径集合中某一条路径path的索引
        /*
                      /       path(pathID为0)
                     /
                    /
        ———————————— —— —— —— path(pathID为1)
                    \
                     \
                      \       path(pathID为2)
                (paths)
        */
        val4 = fscanf(filePtr, "%d", &pathID);
        val5 = fscanf(filePtr, "%f", &point.intensity);

        if (val1 != 1 || val2 != 1 || val3 != 1 || val4 != 1 || val5 != 1)
        {
            printf("\nError reading input files, exit.\n\n");
            exit(1);
        }
        // 遍历paths.ply文件中的点云信息
        if (pathID >= 0 && pathID < pathNum)
        {
            pointSkipCount++;
            // 抛弃前三十个点云信息
            if (pointSkipCount > pointSkipNum)
            {
                paths[pathID]->push_back(point);
                pointSkipCount = 0;
            }
        }
    }

    fclose(filePtr);
}
#endif

/**
 * @brief 读取pathList.ply文件,把路径数据放入pathList和endDirPathList中
 * @target  从指定的 PLY 文件中读取路径列表数据，将数据关联到相关数组中，以便后续的路径规划和使用
                       实现流程和上一个路径数据读取基本一致
 */
void readPathList()
{
    string fileName = pathFolder + "/pathList.ply";

    FILE *filePtr = fopen(fileName.c_str(), "r");
    if (filePtr == NULL)
    {
        printf("\nCannot read input files, exit.\n\n");
        exit(1);
    }

    // pathList.ply文件中共有343个路径，如果不是这个数则抛异常
    if (pathNum != readPlyHeader(filePtr))
    {
        printf("\nIncorrect path number, exit.\n\n");
        exit(1);
    }

    int val1, val2, val3, val4, val5, pathID, groupID;
    float endX, endY, endZ;
    // todo pathList.ply的格式是什么，其中endX,endY,endZ,pathID各是什么意思
    // 格式可以不用管，endX,endY,endZ表示这条路径终点的坐标。
    // ldq: 下图可能不准，pathNum应该是一个方向(10度)343条路径，343条路径分成7组 groupID
    for (int i = 0; i < pathNum; i++)
    {
        // end的为每条路径的最后一个路径点
        val1 = fscanf(filePtr, "%f", &endX);
        val2 = fscanf(filePtr, "%f", &endY);
        val3 = fscanf(filePtr, "%f", &endZ);
        /*fwy
                                   /       path(pathID为0)
                                  /
                                 /
       (groupID为0) ———————————— —— —— —— path(pathID为1)
                                 \
                                  \
                                   \       path(pathID为2)
        (中心都是车体)
                                   /       path(pathID为3)
                                  /
                                 /
       (groupID为1) ———————————— —— —— —— path(pathID为4)
                                 \
                                  \
                                   \       path(pathID为5)

        */
        val4 = fscanf(filePtr, "%d", &pathID);
        val5 = fscanf(filePtr, "%d", &groupID);
        // 无法正常读取数据则抛异常
        if (val1 != 1 || val2 != 1 || val3 != 1 || val4 != 1 || val5 != 1)
        {
            printf("\nError reading input files, exit.\n\n");
            exit(1);
        }
        // groupNum最大是7，认为最大只有7组路径，一组49条
        // pathNum最大是343，认为最大只会生成343个路径
        if (pathID >= 0 && pathID < pathNum && groupID >= 0 && groupID < groupNum)
        {
            pathList[pathID] = groupID;
            endDirPathList[pathID] = detourWei * atan2(endY, endX) * 180 / PI;
            // ROS_WARN("detourWei:%f", detourWei);
        }
    }
    // 关闭文件流
    fclose(filePtr);
}

/**
 * @brief 读取correspondences.txt文件，把数据放进中correspondences中
 *         correspondences的意义是路径点和碰撞体素网格的索引关系
 */
void readCorrespondences()
{
    string fileName = pathFolder + "/correspondences.txt";
    FILE *filePtr = fopen(fileName.c_str(), "r");
    if (filePtr == NULL)
    {
        printf("\nCannot read input files, exit.\n\n");
        exit(1);
    }
    int val1, gridVoxelID, pathID;
    /*
     我们可以把体素网格看作量化的、大小固定的点云
     然而，点云在空间中的任何地方能够以浮点像素坐标的形式涵盖无数个点
     体素网格则是一种三维网格，其中的每个单元（或称「体素」）都有固定的大小和离散的坐标
    */
    // gridVoxelID的意义是体素网格的ID索引
    // gridVoxelNum是161*451=72611
    for (int i = 0; i < gridVoxelNum; i++)
    {
        val1 = fscanf(filePtr, "%d", &gridVoxelID);
        if (val1 != 1)
        {
            printf("\nError reading input files, exit.\n\n");
            exit(1);
        }
        while (1)
        {
            val1 = fscanf(filePtr, "%d", &pathID);
            if (val1 != 1)
            {
                printf("\nError reading input files, exit.\n\n");
                exit(1);
            }
            if (pathID != -1)
            {
                // correspondences是161*451
                if (gridVoxelID >= 0 && gridVoxelID < gridVoxelNum && pathID >= 0 && pathID < pathNum)
                {
                    correspondences[gridVoxelID].push_back(pathID);
                    // 以下用于学习时观察correspondences的数据
                    // ROS_WARN("gridVoxelID:%d", gridVoxelID);
                    // ROS_WARN("pathID:%d", pathID);
                    // cv::Mat blankImage(500, 500, CV_8UC3, cv::Scalar(0, 0, 0));
                    // cv::imshow("Blank Image", blankImage);
                    // cv::waitKey(0);
                }
            }
            else
            {
                break;
            }
        }
    }

    fclose(filePtr);
}