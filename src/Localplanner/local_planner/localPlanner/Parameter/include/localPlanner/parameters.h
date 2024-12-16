#pragma once
#include <iostream>
#include <ros/package.h>
#include <ros/ros.h>
#include <string>
#include <yaml-cpp/yaml.h>
using namespace std;

struct Params
{
    double terrainVoxelSize;
    double maxSpeed;
    double vehicleRadius;
    double close_map_time;
    double adjacentRange;
    double pathRangeStep;
    double minSpeedRange;
    double minPathRange;
    double slow_dis;
    // 障碍物高度相关控制
    double obstacleHeightThre; // 最关键的地面分割阈值
    double groundHeightThre; // 计算高度得分的下界
    double costHeightThre;   // 计算高度得分的上界
    double costScore;        // 最小高度惩罚得分
    double dirWeight;        // 只允许路径终点在这个转弯角度内的路径通过
    double goalClearRange;
    // pathScale相关控制
    double defPathScale;
    double minPathScale;
    double pathScaleStep;
    double goalClearRange_global;
    double add_point_radius;
    double vehicleLength;
    double vehicleWidth;
    double add_point_radius_far;

    int dirThre;             // 规划路径组别的根据车头或目标点筛选，全向运动置为150，给一点后退避障的空间，离目标反向走的不考虑
    int pointPerPathThre;    // 同一块体素位置需要至少多少个障碍物点云才被舍弃

    bool pathRangeBySpeed;
    bool checkRotObstacle;
    bool checkObstacle;
    bool use_map;
    bool use_fail_closemap;
    bool usePathScale;
    bool pathCropByGoal; // 是否根据目标点剪裁路径
    bool pathScaleBySpeed;
    bool useCost; // 是否使用评分计算
};

class ParamControl
{
public:
    ParamControl()
    {
        load_params();
    }
    void load_params();
    void update_params();
    inline void reset_params()
    {
        param = param_origin;
    }
    inline Params get_params() const // 设置外部只读
    {
        return param;
    }
    inline void set_add_point_radius(double rate)
    {
        param.add_point_radius = rate * param_origin.add_point_radius;
        param_rate.add_point_radius = rate;
    }
    inline void set_enlarge_dirThre(double enlarge_dir)
    {
        param.dirThre = param_origin.dirThre + enlarge_dir;
        param_rate.dirThre = double(param.dirThre)/double(param_origin.dirThre);
    }
    inline void set_use_map(bool flag)
    {
        param.use_map = flag;
    }

private:
    void print_params();

    bool load_config(const std::string &local_config, const std::string &usual_config);
    void param_rate_init();
    Params param;
    Params param_origin;
    Params param_rate;
    ros::NodeHandle nhPrivate = ros::NodeHandle("~");
    ros::NodeHandle nhUsual = ros::NodeHandle("usualParams");
    ros::NodeHandle nh;

    string local_config;
    string usual_config;
    string robot;
};