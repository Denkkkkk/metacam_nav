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
    param_rate_init();
}

// 遍历初始化所有的参数比例全部为1
void ParamControl::param_rate_init()
{
    param_rate.terrainVoxelSize = 1.0;
    param_rate.maxSpeed = 1.0;
    param_rate.vehicleRadius = 1.0;
    param_rate.close_map_time = 1.0;
    param_rate.adjacentRange = 1.0;
    param_rate.pathRangeStep = 1.0;
    param_rate.minSpeedRange = 1.0;
    param_rate.minPathRange = 1.0;
    param_rate.slow_dis = 1.0;
    // 障碍物高度相关控制
    param_rate.obstacleHeightThre = 1.0; // 最关键的地面分割阈值
    param_rate.groundHeightThre = 1.0;   // 计算高度得分的下界
    param_rate.costHeightThre = 1.0;     // 计算高度得分的上界
    param_rate.costScore = 1.0;          // 最小高度惩罚得分
    param_rate.dirWeight = 1.0;          // 只允许路径终点在这个转弯角度内的路径通过
    param_rate.goalClearRange = 1.0;
    // pathScale相关控制
    param_rate.defPathScale = 1.0;
    param_rate.minPathScale = 1.0;
    param_rate.pathScaleStep = 1.0;
    param_rate.goalClearRange_global = 1.0;
    param_rate.add_point_radius = 1.0;
    param_rate.vehicleLength = 1.0;
    param_rate.vehicleWidth = 1.0;

    param_rate.dirThre = 1.0;          // 规划路径组别的根据车头或目标点筛选，全向运动置为150，给一点后退避障的空间，离目标反向走的不考虑
    param_rate.pointPerPathThre = 1.0; // 同一块体素位置需要至少多少个障碍物点云才被舍弃
    param_rate.add_point_radius_far = 1.0;
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
        param.vehicleWidth = usual_conf["localPlanner"]["vehicleWidth"].as<double>() * param_rate.vehicleWidth;
        param.vehicleLength = usual_conf["localPlanner"]["vehicleLength"].as<double>() * param_rate.vehicleLength;
        param.add_point_radius = usual_conf["localPlanner"]["add_point_radius"].as<double>() * param_rate.add_point_radius;
        param.minPathRange = usual_conf["localPlanner"]["minPathRange"].as<double>() * param_rate.minPathRange;
        param.goalClearRange = usual_conf["localPlanner"]["goalClearRange"].as<double>() * param_rate.goalClearRange;
        param.slow_dis = usual_conf["localPlanner"]["slow_dis"].as<double>();
        param.obstacleHeightThre = usual_conf["localPlanner"]["obstacleHeightThre"].as<double>() * param_rate.obstacleHeightThre;

        param.use_map = usual_conf["usualParams"]["use_map"].as<bool>();
        param.maxSpeed = usual_conf["usualParams"]["maxSpeed"].as<double>() * param_rate.maxSpeed;
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
        param.dirThre = local_conf["localPlanner"]["dirThre"].as<int>() * param_rate.dirThre;
        param.terrainVoxelSize = local_conf["localPlanner"]["terrainVoxelSize"].as<double>() * param_rate.terrainVoxelSize;

        param.close_map_time = local_conf["localPlanner"]["close_map_time"].as<double>() * param_rate.close_map_time;
        param.adjacentRange = local_conf["localPlanner"]["adjacentRange"].as<double>() * param_rate.adjacentRange;
        param.pathRangeStep = local_conf["localPlanner"]["pathRangeStep"].as<double>() * param_rate.pathRangeStep;
        param.minSpeedRange = local_conf["localPlanner"]["minSpeedRange"].as<double>() * param_rate.minSpeedRange;
        // 障碍物高度相关控制
        param.groundHeightThre = local_conf["localPlanner"]["groundHeightThre"].as<double>() * param_rate.groundHeightThre;
        param.costHeightThre = local_conf["localPlanner"]["costHeightThre"].as<double>() * param_rate.costHeightThre;
        param.costScore = local_conf["localPlanner"]["costScore"].as<double>() * param_rate.costScore;
        param.pointPerPathThre = int(local_conf["localPlanner"]["pointPerPathThre"].as<int>() * param_rate.pointPerPathThre);
        // 目标点或转弯等方向信息的控制
        param.dirWeight = int(local_conf["localPlanner"]["dirWeight"].as<int>() * param_rate.dirWeight);
        param.goalClearRange_global = local_conf["localPlanner"]["goalClearRange_global"].as<double>() * param_rate.goalClearRange_global;
        // scale点云收拢比例控制
        param.defPathScale = local_conf["localPlanner"]["defPathScale"].as<double>() * param_rate.defPathScale;
        param.minPathScale = local_conf["localPlanner"]["minPathScale"].as<double>() * param_rate.minPathScale;
        param.pathScaleStep = local_conf["localPlanner"]["pathScaleStep"].as<double>() * param_rate.pathScaleStep;
        param.add_point_radius_far = local_conf["localPlanner"]["add_point_radius_far"].as<double>() * param_rate.add_point_radius_far;

        param.pathScaleBySpeed = local_conf["localPlanner"]["pathScaleBySpeed"].as<bool>();
        param.usePathScale = local_conf["localPlanner"]["usePathScale"].as<bool>();
        param.pathRangeBySpeed = local_conf["localPlanner"]["pathRangeBySpeed"].as<bool>();
        param.useCost = local_conf["localPlanner"]["useCost"].as<bool>();
        param.checkObstacle = local_conf["localPlanner"]["checkObstacle"].as<bool>();
        param.checkRotObstacle = local_conf["localPlanner"]["checkRotObstacle"].as<bool>();
        param.use_fail_closemap = local_conf["localPlanner"]["use_fail_closemap"].as<bool>();
        param.pathCropByGoal = local_conf["localPlanner"]["pathCropByGoal"].as<bool>();
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