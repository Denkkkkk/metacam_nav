#include "localPlanner/parameters.h"

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
        param.vehicleWidth = usual_conf["localPlanner"]["vehicleWidth"].as<double>();
        param.vehicleLength = usual_conf["localPlanner"]["vehicleLength"].as<double>();
        param.add_point_radius = usual_conf["localPlanner"]["add_point_radius"].as<double>();
        param.minPathRange = usual_conf["localPlanner"]["minPathRange"].as<double>();
        param.goalClearRange = usual_conf["localPlanner"]["goalClearRange"].as<double>();
        param.slow_dis = usual_conf["localPlanner"]["slow_dis"].as<double>();
        param.obstacleHeightThre = usual_conf["localPlanner"]["obstacleHeightThre"].as<double>();

        param.use_map = usual_conf["usualParams"]["use_map"].as<bool>();
        param.maxSpeed = usual_conf["usualParams"]["maxSpeed"].as<double>();
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
        // 常用参数
        param.dirThre = local_conf["localPlanner"]["dirThre"].as<int>();
        param.terrainVoxelSize = local_conf["localPlanner"]["terrainVoxelSize"].as<double>();
        param.checkObstacle = local_conf["localPlanner"]["checkObstacle"].as<bool>();
        param.checkRotObstacle = local_conf["localPlanner"]["checkRotObstacle"].as<bool>();
        param.close_map_time = local_conf["localPlanner"]["close_map_time"].as<double>();
        param.use_fail_closemap = local_conf["localPlanner"]["use_fail_closemap"].as<bool>();
        param.adjacentRange = local_conf["localPlanner"]["adjacentRange"].as<double>();
        param.pathRangeStep = local_conf["localPlanner"]["pathRangeStep"].as<double>();
        param.minSpeedRange = local_conf["localPlanner"]["minSpeedRange"].as<double>();
        param.pathRangeBySpeed = local_conf["localPlanner"]["pathRangeBySpeed"].as<bool>();
        // 障碍物高度相关控制
        param.useCost = local_conf["localPlanner"]["useCost"].as<bool>();
        param.groundHeightThre = local_conf["localPlanner"]["groundHeightThre"].as<double>();
        param.costHeightThre = local_conf["localPlanner"]["costHeightThre"].as<double>();
        param.costScore = local_conf["localPlanner"]["costScore"].as<double>();
        param.pointPerPathThre = local_conf["localPlanner"]["pointPerPathThre"].as<int>();
        // 目标点或转弯等方向信息的控制
        param.dirWeight = local_conf["localPlanner"]["dirWeight"].as<int>();
        param.pathCropByGoal = local_conf["localPlanner"]["pathCropByGoal"].as<bool>();
        param.goalClearRange_global = local_conf["localPlanner"]["goalClearRange_global"].as<double>();
        // scale点云收拢比例控制
        param.usePathScale = local_conf["localPlanner"]["usePathScale"].as<bool>();
        param.defPathScale = local_conf["localPlanner"]["defPathScale"].as<double>();
        param.minPathScale = local_conf["localPlanner"]["minPathScale"].as<double>();
        param.pathScaleStep = local_conf["localPlanner"]["pathScaleStep"].as<double>();
        param.pathScaleBySpeed = local_conf["localPlanner"]["pathScaleBySpeed"].as<bool>();
    }
    catch (YAML::Exception &e)
    {
        std::cerr << "YAML Parsing Error: " << e.what() << std::endl;
    }
    return true;
}

void ParamControl::update_params()
{
    // nhUsual.getParam("maxSpeed", param.maxSpeed);
    // nhUsual.getParam("use_map", param.use_map);
    // nhPrivate.getParam("terrainVoxelSize", param.terrainVoxelSize);
    // nhPrivate.getParam("checkRotObstacle", param.checkRotObstacle);
    // nhPrivate.getParam("vehicleRadius", param.vehicleRadius);
    // nhPrivate.getParam("checkObstacle", param.checkObstacle);
    // nhPrivate.getParam("adjacentRange", param.adjacentRange);
    // nhPrivate.getParam("obstacleHeightThre", param.obstacleHeightThre);
    // nhPrivate.getParam("groundHeightThre", param.groundHeightThre);
    // nhPrivate.getParam("costHeightThre", param.costHeightThre);
    // nhPrivate.getParam("costScore", param.costScore);
    // nhPrivate.getParam("useCost", param.useCost);
    // nhPrivate.getParam("pointPerPathThre", param.pointPerPathThre);
    // nhPrivate.getParam("dirWeight", param.dirWeight);
    // nhPrivate.getParam("dirThre", param.dirThre);
    // nhPrivate.getParam("minPathScale", param.minPathScale);
    // nhPrivate.getParam("pathScaleStep", param.pathScaleStep);
    // nhPrivate.getParam("pathScaleBySpeed", param.pathScaleBySpeed);
    // nhPrivate.getParam("minPathRange", param.minPathRange);
    // nhPrivate.getParam("pathRangeStep", param.pathRangeStep);
    // nhPrivate.getParam("pathRangeBySpeed", param.pathRangeBySpeed);
    // nhPrivate.getParam("pathCropByGoal", param.pathCropByGoal);
    // nhPrivate.getParam("goalClearRange", param.goalClearRange);
    // nhPrivate.getParam("slow_dis", param.slow_dis);
    // nhPrivate.getParam("defPathScale", param.defPathScale);
    // nhPrivate.getParam("usePathScale", param.usePathScale);
    // nhPrivate.getParam("minSpeedRange", param.minSpeedRange);
    // nhPrivate.getParam("close_map_time", param.close_map_time);
    // nhPrivate.getParam("use_fail_closemap", param.use_fail_closemap);
    // nhPrivate.getParam("goalClearRange_global", param.goalClearRange_global);
    // nhPrivate.getParam("add_point_radius", param.add_point_radius);
    // nhPrivate.getParam("vehicleLength", param.vehicleLength);
    // nhPrivate.getParam("vehicleWidth", param.vehicleWidth);

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
    cout << "Parameters: " << endl;
    cout << "terrainVoxelSize: " << param.terrainVoxelSize << endl;
    cout << "checkRotObstacle: " << (param.checkRotObstacle ? "true" : "false") << endl;
    cout << "maxSpeed: " << param.maxSpeed << endl;
    cout << "vehicleRadius: " << param.vehicleRadius << endl;
    cout << "checkObstacle: " << (param.checkObstacle ? "true" : "false") << endl;
    cout << "use_map: " << (param.use_map ? "true" : "false") << endl;
    cout << "close_map_time: " << param.close_map_time << endl;
    cout << "use_fail_closemap: " << (param.use_fail_closemap ? "true" : "false") << endl;
    cout << "adjacentRange: " << param.adjacentRange << endl;
    cout << "pathRangeStep: " << param.pathRangeStep << endl;
    cout << "minSpeedRange: " << param.minSpeedRange << endl;
    cout << "minPathRange: " << param.minPathRange << endl;
    cout << "pathRangeBySpeed: " << (param.pathRangeBySpeed ? "true" : "false") << endl;
    cout << "slow_dis: " << param.slow_dis << endl;
    cout << "pathScaleBySpeed: " << (param.pathScaleBySpeed ? "true" : "false") << endl;
    cout << "obstacleHeightThre: " << param.obstacleHeightThre << endl;
    cout << "useCost: " << (param.useCost ? "true" : "false") << endl;
    cout << "groundHeightThre: " << param.groundHeightThre << endl;
    cout << "costHeightThre: " << param.costHeightThre << endl;
    cout << "costScore: " << param.costScore << endl;
    cout << "pointPerPathThre: " << param.pointPerPathThre << endl;
    cout << "dirWeight: " << param.dirWeight << endl;
    cout << "dirThre: " << param.dirThre << endl;
    cout << "pathCropByGoal: " << (param.pathCropByGoal ? "true" : "false") << endl;
    cout << "goalClearRange: " << param.goalClearRange << endl;
    cout << "usePathScale: " << (param.usePathScale ? "true" : "false") << endl;
    cout << "defPathScale: " << param.defPathScale << endl;
    cout << "minPathScale: " << param.minPathScale << endl;
    cout << "pathScaleStep: " << param.pathScaleStep << endl;
    cout << "goalClearRange_global: " << param.goalClearRange_global << endl;
    cout << "add_point_radius: " << param.add_point_radius << endl;
    cout << "vehicleLength: " << param.vehicleLength << endl;
    cout << "vehicleWidth: " << param.vehicleWidth << endl;
}