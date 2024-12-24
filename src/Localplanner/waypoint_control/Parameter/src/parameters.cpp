#include "waypoint_control/parameters.h"

void ParamControl::load_params()
{
    // 加载路径
    nh.getParam("robot", robot);
    local_config = ros::package::getPath("nav_gplanner_start") + "/config/" + "system_gplanner.yaml";
    usual_config = ros::package::getPath("param_config") + "/config/nav_config.yaml";
    // 打印路径
    std::cout << "local_config: " << local_config << std::endl;
    std::cout << "usual_config: " << usual_config << std::endl;

    load_config(local_config, usual_config);
    param_rate_init();
}

bool ParamControl::load_config(const std::string &local_config, const std::string &usual_config)
{
    param_rate_init();
    // 从nav_config.yaml读取 local_planner 参数
    // 读取参数
    std::cout << "[YAML] Loading " << ros::this_node::getName() << " usual" << " parameters... " << std::endl;
    try
    {
        YAML::Node usual_conf = YAML::LoadFile(usual_config);
    }
    catch (YAML::BadFile &e)
    {
        std::cerr << "YAML Parsing Error: " << e.what() << std::endl;
    }

    // 从local_planner.yaml读取 local_planner 参数
    std::cout << "[YAML] Loading " << ros::this_node::getName() << " local" << " parameters... " << std::endl;
    try
    {
        YAML::Node local_conf = YAML::LoadFile(local_config);
    }
    catch (YAML::Exception &e)
    {
        std::cerr << "YAML Parsing Error: " << e.what() << std::endl;
    }
    param_origin = param;
    return true;
}

void ParamControl::update_params()
{
    // 创建静态计数器
    static int count = 1;
    // 每隔10次更新一次参数
    if (count % 10 == 0)
    {
        load_config(local_config, usual_config);
        count = 0;
    }
    count++;
    // print_params();
}

void ParamControl::print_params()
{
    // 打印 Params 结构体中的各个参数

}