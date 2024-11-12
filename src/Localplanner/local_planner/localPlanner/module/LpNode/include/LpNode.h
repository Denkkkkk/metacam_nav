#pragma once
#include "Parts.h"
#include "localPlanner/parameters.h"
#include <cmath>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <pcl/io/pcd_io.h> //pcd 读写类相关的头文件。
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <time.h>
#include <visualization_msgs/Marker.h>
class LpNode
{
private:
    ros::NodeHandle nh;
    ros::NodeHandle nhPrivate = ros::NodeHandle("~");
    // 回调函数，提取里程计信息，更新车体的位姿信息
    ros::Subscriber subOdometry;
    // 订阅地形点云/terrain_map，对地形点云的处理
    ros::Subscriber subTerrainCloud;
    // 回调获取目标点
    ros::Subscriber subGoal;
    // 控制joySpeed
    ros::Subscriber subSpeed;
    // 处理导航的边界点，目的是将下采样（减少计算量）后的边界点数据，重新进行插值平滑处理
    ros::Subscriber subAddCloud;
    // 订阅到碰撞检查的信号，对检查标志位置1
    ros::Subscriber subCheckObstacle;
    // 订阅动态调参器
    ros::Subscriber subLplannerData;
    // 虚拟车头方向
    ros::Subscriber subVirHeadDir;
    // 关闭局部地图标志位
    ros::Subscriber subCloseMap;
    // 已经到达目的地
    ros::Subscriber subGetGoal;
    // 全局目标点
    ros::Subscriber subGlobal_point;

    // 发布局部规划路径
    ros::Publisher pubPath;
    ros::Publisher pubGlobalPath;
    // 发布用于规划的点云
    ros::Publisher pubPannerAtuCloud;
    // 发布减速命令
    ros::Publisher pubSlowDown;
    ros::Publisher pubMap;
    ros::Publisher pubMarker;
    ros::Publisher pubVirHeadDir;
    ros::Publisher pubAddPoints;
    int Xbias;
    int Ybias;
    std::string usual_pcd_path;
    pcl::PointCloud<pcl::PointXYZI>::Ptr terrainMapRecord_pcl;
    sensor_msgs::PointCloud2 terrainMapRecord;
    std::string robotFrame;
    visualization_msgs::Marker marker; // 可视化的形状
    uint32_t shape;
    float goalClearCenter_x = -2.2;
    float goalClearCenter_y = 3.5;
    bool get_addedObstacles = false;
    geometry_msgs::PoseStamped goal_point;

#if PLOTPATHSET == 1
    ros::Publisher pubFreePaths;
#endif

    // 提取里程计传感器信息，更新车体的位姿信息
    void odometryHandler(const nav_msgs::Odometry::ConstPtr &odom);
    // 订阅地形点云/terrain_map，对地形点云的处理
    void terrainCloudHandler(const sensor_msgs::PointCloud2ConstPtr &terrainCloud2);
    // 回调获取目标点
    void goalHandler(const geometry_msgs::PoseStamped::ConstPtr &goal);
    // 控制joySpeed
    void speedHandler(const std_msgs::Float32::ConstPtr &speed);
    // 额加的雷达避障信息
    void addCloudHandler(const sensor_msgs::PointCloud2ConstPtr &addPoints);
    // 控制是否开启障碍物检测的状态
    void checkObstacleHandler(const std_msgs::Bool::ConstPtr &checkObs);
    void virHeadDirHandler(const std_msgs::Float32::ConstPtr &msg);
    void closeMapHandler(const std_msgs::Bool::ConstPtr &msg);
    void globalPointHandler(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void load_pcd_map();

public:
    LpNode();
    ~LpNode()
    {
        delete lctlPtr;
    };
    ParamControl *lctlPtr;
    void read_pathfile(); // 初始化容器，并读取保存各个path数据文件
    /*
        以下三个转换连着使用，调用transform_allCloud();
    */
    void transform_plannerCloud();        // 以车头为x方向重新建系，算点云的坐标。并根据距离，地形分析下的高度上下限筛选点云
    void transform_addedObstaclesCloud(); // 以车头重新建系，算其他额外传感器点云的坐标。并根据距离筛选
    void transform_allCloud();            // 将以上三个转换一起调用,转换所有点云到车体头坐标系，并按照距离过滤

    void transform_goal();     // 在自动模式下, 将目标点转移到车体坐标系下,并记录下目标相对车体的yaw角
    void local_planner();      // 正式进行局部路径规划寻找
    void fail_local_planner(); // 路径规划失败
    /*
        以下是local_planner()中封装的函数模块
    */
    void pathRange_from_speed(); // 将路径范围根据速度进行动态调整
    void clear_Lists_score();    // 规划前清空路径点集和分数
    //! 到当前点云的网格下，将障碍物信息同步到该网格的路径中（清除路径或根据点云高度设置惩罚值),现在还没筛选出路径，根据网格来限制来卡死这个位置，不再对这区域规划
    //! rotDir是相对目标点方向的即将转向角度(10度一单位)，x2是车体坐标系下的点云坐标
    void grid_Synchronize_obstacles_to_paths(int rotDir, float x2, float y2, float h);
    void local_diff_limitTurn(int x, int y);                     // 针对差速底盘存在额外转弯半径的转弯角度限制
    void count_PathPerGroupScore(int pathId, int rotDir);        // 正式以路径组的形式计算得分，有三个得分标准，得分越高路径越优
    void select_from_dir(float &maxScore, int &selectedGroupID); // 根据最小转向角进行路径筛选
    void pub_allFreePath();                                      // 以车头方向为参考系，发布所有的无障碍物路径
    void pubPath_fromStartPaths(int &selectedGroupID);           // 找到最优路径组初始方向后，根据方向从第一次采样中筛选出对应方向上的路径点，并发布
    void pub_Map();                                              // 发布地形点云
    void makerInit();
    void close_map();
    double close_map_begin = -1.0;
    bool need_close_map = false;
    bool need_read_path = false;
    double pathScale;
    double actual_goalClearRange;
};