#pragma once
#include <iostream>
#include <ros/ros.h>
#include <string>

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
    std::string cmdTopicdTopic;
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
};

class ParamControl
{
public:
    void load_params();
    void update_params();
    inline Params get_params() const // 设置外部只读
    {
        return param;
    }

private:
    ros::NodeHandle nhPrivate = ros::NodeHandle("~");
    ros::NodeHandle nhPrivate_actual = ros::NodeHandle("pathFollower_actual");
    ros::NodeHandle nhUsual = ros::NodeHandle("usualParams");
    ros::NodeHandle nh;

    Params param;
    Params param_origin;
};