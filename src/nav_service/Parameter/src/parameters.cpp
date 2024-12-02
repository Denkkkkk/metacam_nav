#include "nav_service/parameters.h"

void ParamControl::load_params()
{
    // 从参数服务器中获取参数
    nh.getParam("/pathFollower/endGoalDis", param.endGoalDis);
    nh.param("/usualParams/use_map", param.use_prior_path, false);
    // 读取文件
    nhPrivate.getParam("prior_path_file", param.prior_path_file);
    if (param.use_prior_path)
    {
        param.prior_path = readCoordinates(param.prior_path_file);
    }
    param_origin = param;
}

void ParamControl::update_params()
{
    // 允许直接更新的参数
    nh.getParam("/pathFollower/endGoalDis", param.endGoalDis);
    nh.getParam("/usualParams/use_map", param.use_prior_path);

    // 重新读取文件
    static int num = 0;
    if (param.use_prior_path && num < 0)
    {
        param.prior_path = readCoordinates(param.prior_path_file);
        num = 10;
    }
    num--;
}

std::vector<Coordinate> ParamControl::readCoordinates(const std::string &filename)
{
    std::vector<Coordinate> coordinates;
    std::ifstream file(filename);

    if (!file)
    {
        ROS_ERROR("Error opening file!");
        return coordinates;
    }
    std::string line;
    while (std::getline(file, line))
    {
        // 忽略以 # 开头的注释行
        if (line.empty() || line[0] == '#')
        {
            continue;
        }
        std::stringstream ss(line);
        Coordinate coord;
        char delimiter; // 用于处理逗号分隔
        if (ss >> coord.x >> delimiter >> coord.y >> delimiter >> coord.yaw)
        {
            coordinates.push_back(coord);
        }
    }
    file.close();
    return coordinates;
}