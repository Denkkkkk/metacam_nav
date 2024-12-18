#include "pathFollower/parameters.h"

void ParamControl::load_params()
{
    // 加载路径
    nh.getParam("robot", robot);
    local_config = ros::package::getPath("local_planner") + "/config/" + robot + ".yaml";
    usual_config = ros::package::getPath("param_config") + "/config/nav_config.yaml";
    // 打印路径
    std::cout << "local_config: " << local_config << std::endl;
    std::cout << "usual_config: " << usual_config << std::endl;

    load_config(local_config, usual_config);
    param_origin = param;
}

bool ParamControl::load_config(const std::string &local_config, const std::string &usual_config)
{
    // 从nav_config.yaml读取 local_planner 参数
    // 读取参数
    std::cout << "[YAML] Loading " << ros::this_node::getName() << " usual" << " parameters... " << std::endl;
    try
    {
        YAML::Node usual_conf = YAML::LoadFile(usual_config);
        // 从 usual_conf 中提取各项参数
        param.endGoalDis = usual_conf["pathFollower"]["endGoalDis"].as<double>();
        param.use_closeGoal_direct = usual_conf["pathFollower"]["use_closeGoal_direct"].as<bool>();
        param.useCloudSlowDown = usual_conf["pathFollower"]["useCloudSlowDown"].as<bool>();
        param.slowdown_rate = usual_conf["pathFollower"]["slowdown_rate"].as<double>();
        param.minSpeed = usual_conf["pathFollower"]["minSpeed"].as<double>();
        param.maxAddAccel = usual_conf["pathFollower"]["maxAddAccel"].as<double>();
        param.maxSlowAccel = usual_conf["pathFollower"]["maxSlowAccel"].as<double>();
        param.use_getgoal_yaw = usual_conf["pathFollower"]["use_getgoal_yaw"].as<bool>();
        param.getgoal_yaw = usual_conf["pathFollower"]["getgoal_yaw"].as<double>();
        param.vehicle_stop_range = usual_conf["pathFollower"]["vehicle_stop_range"].as<double>();

        param.maxSpeed = usual_conf["usualParams"]["maxSpeed"].as<double>();
        param.use_map = usual_conf["usualParams"]["use_map"].as<bool>();

        param.obstacleHeightThre = usual_conf["localPlanner"]["obstacleHeightThre"].as<double>();
        param.localPlanner_slow_dis = usual_conf["localPlanner"]["slow_dis"].as<double>();
        
    }
    catch (YAML::BadFile &e)
    {
        std::cerr << "YAML Parsing Error: " << e.what() << std::endl;
    }

    std::cout << "[YAML] Loading " << ros::this_node::getName() << " local" << " parameters... " << std::endl;
    try
    {
        YAML::Node local_conf = YAML::LoadFile(local_config);
        // Sports parameters
        param.yawRateGain = local_conf["pathFollower"]["yawRateGain"].as<double>();
        param.stopYawRateGain = local_conf["pathFollower"]["stopYawRateGain"].as<double>();
        param.maxYawRate = local_conf["pathFollower"]["maxYawRate"].as<double>();
        param.maxStopYawRate = local_conf["pathFollower"]["maxStopYawRate"].as<double>();
        param.dirDiffThre_slow = local_conf["pathFollower"]["dirDiffThre_slow"].as<double>();
        param.dirDiffThre_keep = local_conf["pathFollower"]["dirDiffThre_keep"].as<double>();
        param.quick_turn_speed = local_conf["pathFollower"]["quick_turn_speed"].as<double>();
        // Get goal parameters
        param.close_direct_speed = local_conf["pathFollower"]["close_direct_speed"].as<double>();
        param.closeGoal_direct_dis = local_conf["pathFollower"]["closeGoal_direct_dis"].as<double>();
        param.goalSlowDisThre = local_conf["pathFollower"]["goalSlowDisThre"].as<double>();
        param.getGoal_speed = local_conf["pathFollower"]["getGoal_speed"].as<double>();
        // Generally fixed parameters
        param.lookAheadDis = local_conf["pathFollower"]["lookAheadDis"].as<double>();
        param.useLoaclSlow = local_conf["pathFollower"]["useLoaclSlow"].as<bool>();
        param.endPathDis = local_conf["pathFollower"]["endPathDis"].as<double>();
        param.pathSlowDisThre = local_conf["pathFollower"]["pathSlowDisThre"].as<double>();
        param.getPath_speed = local_conf["pathFollower"]["getPath_speed"].as<double>();
        param.path_zero_bias = local_conf["pathFollower"]["path_zero_bias"].as<double>();
        param.goal_zero_bias = local_conf["pathFollower"]["goal_zero_bias"].as<double>();
        param.cloudSlow_minSpeed = local_conf["pathFollower"]["cloudSlow_minSpeed"].as<double>();
        param.curvature = local_conf["pathFollower"]["curvature"].as<double>();
        param.goal_path_direct = local_conf["pathFollower"]["goal_path_direct"].as<bool>();
        param.use_MIDPlanning_slow = local_conf["pathFollower"]["use_MIDPlanning_slow"].as<bool>();
        param.MIDPlanning_slow_rate = local_conf["pathFollower"]["MIDPlanning_slow_rate"].as<double>();
        param.MIDPlanning_minSpeed = local_conf["pathFollower"]["MIDPlanning_minSpeed"].as<double>();
        param.use_virtual_head = local_conf["pathFollower"]["use_virtual_head"].as<bool>();

    }
    catch (YAML::BadFile &e)
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
    nh.param("localPlanner/pathRange", param.localPlanner_pathRange, 3.0);
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
    // 打印参数
    std::cout << "lookAheadDis: " << param.lookAheadDis << std::endl;
    std::cout << "maxSpeed: " << param.maxSpeed << std::endl;
    std::cout << "useLoaclSlow: " << (param.useLoaclSlow ? "true" : "false") << std::endl;
    std::cout << "endPathDis: " << param.endPathDis << std::endl;
    std::cout << "pathSlowDisThre: " << param.pathSlowDisThre << std::endl;
    std::cout << "getPath_speed: " << param.getPath_speed << std::endl;
    std::cout << "path_zero_bias: " << param.path_zero_bias << std::endl;
    std::cout << "goalSlowDisThre: " << param.goalSlowDisThre << std::endl;
    std::cout << "getGoal_speed: " << param.getGoal_speed << std::endl;
    std::cout << "goal_zero_bias: " << param.goal_zero_bias << std::endl;
    std::cout << "dirDiffThre_slow: " << param.dirDiffThre_slow << std::endl;
    std::cout << "dirDiffThre_keep: " << param.dirDiffThre_keep << std::endl;
    std::cout << "endGoalDis: " << param.endGoalDis << std::endl;
    std::cout << "useCloudSlowDown: " << (param.useCloudSlowDown ? "true" : "false") << std::endl;
    std::cout << "cloudSlow_minSpeed: " << param.cloudSlow_minSpeed << std::endl;
    std::cout << "minSpeed: " << param.minSpeed << std::endl;
    std::cout << "curvature: " << param.curvature << std::endl;
    std::cout << "maxAddAccel: " << param.maxAddAccel << std::endl;
    std::cout << "maxSlowAccel: " << param.maxSlowAccel << std::endl;
    std::cout << "yawRateGain: " << param.yawRateGain << std::endl;
    std::cout << "stopYawRateGain: " << param.stopYawRateGain << std::endl;
    std::cout << "maxYawRate: " << param.maxYawRate << std::endl;
    std::cout << "maxStopYawRate: " << param.maxStopYawRate << std::endl;
    std::cout << "goal_path_direct: " << (param.goal_path_direct ? "true" : "false") << std::endl;
    std::cout << "use_MIDPlanning_slow: " << (param.use_MIDPlanning_slow ? "true" : "false") << std::endl;
    std::cout << "MIDPlanning_slow_rate: " << param.MIDPlanning_slow_rate << std::endl;
    std::cout << "MIDPlanning_minSpeed: " << param.MIDPlanning_minSpeed << std::endl;
    std::cout << "use_closeGoal_direct: " << (param.use_closeGoal_direct ? "true" : "false") << std::endl;
    std::cout << "closeGoal_direct_dis: " << param.closeGoal_direct_dis << std::endl;
    std::cout << "quick_turn_speed: " << param.quick_turn_speed << std::endl;
    std::cout << "close_direct_speed: " << param.close_direct_speed << std::endl;
    std::cout << "use_virtual_head: " << (param.use_virtual_head ? "true" : "false") << std::endl;
    std::cout << "use_map: " << (param.use_map ? "true" : "false") << std::endl;
    std::cout << "use_getgoal_yaw: " << (param.use_getgoal_yaw ? "true" : "false") << std::endl;
    std::cout << "getgoal_yaw: " << param.getgoal_yaw << std::endl;
    std::cout << "slowdown_rate: " << param.slowdown_rate << std::endl;
    std::cout << "localPlanner_pathRange: " << param.localPlanner_pathRange << std::endl;
    std::cout << "obstacleHeightThre: " << param.obstacleHeightThre << std::endl;
}