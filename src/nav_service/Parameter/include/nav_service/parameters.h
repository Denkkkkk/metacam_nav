#pragma once
#include <fstream>
#include <iostream>
#include <nav_service/parameters.h>
#include <ros/ros.h>
#include <sstream>
#include <string>

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
    double endGoalDis = 0.6;
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
    ros::NodeHandle nhPrivate = ros::NodeHandle("~");
    ros::NodeHandle nh;

    Params param;
    Params param_origin;
    std::vector<Coordinate> readCoordinates(const std::string &filename);
};