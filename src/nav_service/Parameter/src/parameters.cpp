#include "nav_service/parameters.h"

void ParamControl::load_params()
{
    // 加载路径
    usual_config = ros::package::getPath("param_config") + "/config/nav_config.yaml";

    std::string path_file_name;
    try
    {
        YAML::Node usual_conf = YAML::LoadFile(usual_config);
        path_file_name = usual_conf["nav_service"]["path_file_name"].as<std::string>();
    }
    catch (YAML::BadFile &e)
    {
        std::cerr << "YAML Parsing Error: " << e.what() << std::endl;
    }
    param.prior_path_file = ros::package::getPath("param_config") + "/config/prior_path/" + path_file_name;

    load_config(usual_config);

    param_origin = param;
}

bool ParamControl::load_config(const std::string &usual_config)
{
    if (param.use_prior_path)
    {
        param.prior_path = readCoordinates(param.prior_path_file);
    }
    // 从nav_config.yaml读取 local_planner 参数
    // 读取参数
    std::cout << "[YAML] Loading " << ros::this_node::getName() << " usual" << " parameters... " << std::endl;
    try
    {
        YAML::Node usual_conf = YAML::LoadFile(usual_config);
        param.endGoalDis = usual_conf["pathFollower"]["endGoalDis"].as<double>();
        param.use_prior_path = usual_conf["usualParams"]["use_prior_path"].as<bool>();
        param.use_relocalization = usual_conf["usualParams"]["use_relocalization"].as<bool>();
        param.endGoal_stopTime = usual_conf["nav_service"]["endGoal_stopTime"].as<double>();
    }
    catch (YAML::BadFile &e)
    {
        std::cerr << "YAML Parsing Error: " << e.what() << std::endl;
    }
    return true;
}

void ParamControl::update_params()
{

    // 创建静态计数器
    static int count = 1;
    // 每隔10次更新一次参数
    if (count % 10 == 0)
    {
        load_config(usual_config);
    }
    count++;

    print_params();
}

void ParamControl::print_params()
{
    std::cout << "[YAML] " << ros::this_node::getName() << " parameters:" << std::endl;
    // std::cout << "[YAML] " << "endGoalDis: " << param.endGoalDis << std::endl;
    std::cout << "[YAML] " << "endGoal_stopTime: " << param.endGoal_stopTime << std::endl;
    std::cout << "[YAML] " << "use_prior_path: " << (param.use_prior_path ? "true" : "false") << std::endl;
    std::cout << "[YAML] " << "use_relocalization: " << (param.use_relocalization ? "true" : "false") << std::endl;
    std::cout << std::endl;
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