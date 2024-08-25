#pragma once
#include <iostream>
#include <ros/ros.h>
#include <string>
using namespace std;

struct Params
{
    double vehicleRadius;
    bool checkObstacle;
    bool useMap;
    double close_map_time;
    bool use_fail_closemap;
    double adjacentRange;
    double pathRangeStep;
    double minSpeedRange;
    double minPathRange;
    bool pathRangeBySpeed;
    double slow_dis;
    bool pathScaleBySpeed;
    // 障碍物高度相关控制
    double obstacleHeightThre; // 最关键的地面分割阈值
    bool useCost;              // 是否使用评分计算
    double groundHeightThre;   // 计算高度得分的下界
    double costHeightThre;     // 计算高度得分的上界
    double costScore;          // 最小高度惩罚得分
    int pointPerPathThre;      // 同一块体素位置需要至少多少个障碍物点云才被舍弃
    double dirWeight;          // 只允许路径终点在这个转弯角度内的路径通过
    double dirThre;            // 规划路径组别的根据车头或目标点筛选，全向运动置为150，给一点后退避障的空间，离目标反向走的不考虑
    bool pathCropByGoal;       // 是否根据目标点剪裁路径
    double goalClearRange;
    // pathScale相关控制
    bool usePathScale;
    double defpathScale;
    double minPathScale;
    double pathScaleStep;
    bool use_pcd_close;
    double goalClearRange_global;
};

class ParamControl
{
public:
    void load_params();
    void update_params();

    Params param;
    Params param_origin;
    ros::NodeHandle nhPrivate = ros::NodeHandle("~");
    ros::NodeHandle nh;
};