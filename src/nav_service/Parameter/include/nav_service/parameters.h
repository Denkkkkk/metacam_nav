#pragma once
#include <fstream>
#include <iostream>
#include <ros/package.h>
#include <ros/ros.h>
#include <sstream>
#include <string>
#include <yaml-cpp/yaml.h>

// prior_path的路点表示
struct Coordinate
{
    float x, y, z;
    float yaw;
};

struct Params
{
    bool use_prior_path;
    std::vector<Coordinate> prior_path;
    std::string prior_path_file;
    double endGoalDis = 0.5;
    double endGoal_stopTime = 5.0;
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
    inline Params get_params() const // 设置外部只读
    {
        return param;
    }

private:
    void print_params();
    bool load_config(const std::string &usual_config);

    ros::NodeHandle nhPrivate = ros::NodeHandle("~");
    ros::NodeHandle nh;

    Params param;
    Params param_origin;
    std::vector<Coordinate> readCoordinates(const std::string &filename);

    std::string local_config;
    std::string usual_config;
    std::string robot;
};