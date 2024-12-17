#pragma once
#include <iostream>
#include <ros/package.h>
#include <ros/ros.h>
#include <string>
#include <yaml-cpp/yaml.h>
using namespace std;

struct Params
{
    double lookAheadDis;
    double maxSpeed;
    bool useLoaclSlow;
    double endPathDis;
    double pathSlowDisThre;
    double getPath_speed;
    double path_zero_bias;
    double goalSlowDisThre;
    double getGoal_speed;
    double goal_zero_bias;
    double dirDiffThre_slow;
    double dirDiffThre_keep;
    double endGoalDis;
    bool useCloudSlowDown;
    double cloudSlow_minSpeed;
    double minSpeed;
    double curvature;
    int slowBegin;
    double maxAddAccel;
    double maxSlowAccel;
    double yawRateGain;
    double stopYawRateGain;
    double maxYawRate;
    double maxStopYawRate;
    bool goal_path_direct;
    bool use_MIDPlanning_slow;
    double MIDPlanning_slow_rate;
    double MIDPlanning_minSpeed;
    bool use_closeGoal_direct;
    double closeGoal_direct_dis;
    double quick_turn_speed;
    double close_direct_speed;
    bool use_virtual_head;
    bool use_move_base;
    bool use_getgoal_yaw;
    double getgoal_yaw;
    double slowdown_rate;
    double localPlanner_pathRange;
    double obstacleHeightThre;
    double localPlanner_slow_dis;
    double vehicle_stop_range;
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

private:
    bool load_config(const std::string &local_config, const std::string &usual_config);
    void print_params();
    Params param;
    Params param_origin;
    ros::NodeHandle nhPrivate = ros::NodeHandle("~");
    ros::NodeHandle nhUsual = ros::NodeHandle("usualParams");
    ros::NodeHandle nh;

    string local_config;
    string usual_config;
    string robot;
};