#pragma once
#include <iostream>
#include <ros/package.h>
#include <ros/ros.h>
#include <string>
#include <yaml-cpp/yaml.h>
using namespace std;

struct Params
{

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