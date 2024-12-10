#include "localPlanner/parameters.h"

void ParamControl::load_params()
{
    // 加载通用参数
    nhUsual.getParam("maxSpeed", param.maxSpeed);
    nhUsual.getParam("use_map", param.useMap);

    nhPrivate.getParam("terrainVoxelSize", param.terrainVoxelSize);
    nhPrivate.getParam("checkRotObstacle", param.checkRotObstacle);
    nhPrivate.getParam("vehicleRadius", param.vehicleRadius);
    nhPrivate.getParam("checkObstacle", param.checkObstacle);
    nhPrivate.getParam("adjacentRange", param.adjacentRange);
    nhPrivate.getParam("obstacleHeightThre", param.obstacleHeightThre);
    nhPrivate.getParam("groundHeightThre", param.groundHeightThre);
    nhPrivate.getParam("costHeightThre", param.costHeightThre);
    nhPrivate.getParam("costScore", param.costScore);
    nhPrivate.getParam("useCost", param.useCost);
    nhPrivate.getParam("pointPerPathThre", param.pointPerPathThre);
    nhPrivate.getParam("dirWeight", param.dirWeight);
    nhPrivate.getParam("dirThre", param.dirThre);
    nhPrivate.getParam("minPathScale", param.minPathScale);
    nhPrivate.getParam("pathScaleStep", param.pathScaleStep);
    nhPrivate.getParam("pathScaleBySpeed", param.pathScaleBySpeed);
    nhPrivate.getParam("minPathRange", param.minPathRange);
    nhPrivate.getParam("pathRangeStep", param.pathRangeStep);
    nhPrivate.getParam("pathRangeBySpeed", param.pathRangeBySpeed);
    nhPrivate.getParam("pathCropByGoal", param.pathCropByGoal);
    nhPrivate.getParam("goalClearRange", param.goalClearRange);
    nhPrivate.getParam("slow_dis", param.slow_dis);
    nhPrivate.getParam("defPathScale", param.defPathScale);
    nhPrivate.getParam("usePathScale", param.usePathScale);
    nhPrivate.getParam("minSpeedRange", param.minSpeedRange);
    nhPrivate.getParam("close_map_time", param.close_map_time);
    nhPrivate.getParam("use_fail_closemap", param.use_fail_closemap);
    nhPrivate.getParam("goalClearRange_global", param.goalClearRange_global);
    nhPrivate.getParam("add_point_radius", param.add_point_radius);
    nhPrivate.getParam("vehicleLength", param.vehicleLength);
    nhPrivate.getParam("vehicleWidth", param.vehicleWidth);
    param_origin = param;
}

bool ParamControl::load_config(const std::string &config)
{
    
    std::cout << "[YAML] Loading paths..." << std::endl;
    std::cout << "[YAML] Loading " << ros::this_node::getName() << "parameters... " << std::endl;

    // 从nav_config.yaml读取 local_planner 参数
    YAML::Node conf = YAML::LoadFile(config);
    double vehicleWidth = conf["localPlanner"]["vehicleWidth"].as<double>();
    double vehicleLength = conf["localPlanner"]["vehicleLength"].as<double>();
    double addPointRadius = conf["localPlanner"]["add_point_radius"].as<double>();
    double minPathRange = conf["localPlanner"]["minPathRange"].as<double>();
    double goalClearRange = conf["localPlanner"]["goalClearRange"].as<double>();
    double slowDis = conf["localPlanner"]["slow_dis"].as<double>();
    double obstacleHeightThre = conf["localPlanner"]["obstacleHeightThre"].as<double>();

    // 从local_planner.yaml读取 local_planner 参数
    

    return true;
}

void ParamControl::update_params()
{
    nhUsual.getParam("maxSpeed", param.maxSpeed);
    nhUsual.getParam("use_map", param.useMap);

    nhPrivate.getParam("terrainVoxelSize", param.terrainVoxelSize);
    nhPrivate.getParam("checkRotObstacle", param.checkRotObstacle);
    nhPrivate.getParam("vehicleRadius", param.vehicleRadius);
    nhPrivate.getParam("checkObstacle", param.checkObstacle);
    nhPrivate.getParam("adjacentRange", param.adjacentRange);
    nhPrivate.getParam("obstacleHeightThre", param.obstacleHeightThre);
    nhPrivate.getParam("groundHeightThre", param.groundHeightThre);
    nhPrivate.getParam("costHeightThre", param.costHeightThre);
    nhPrivate.getParam("costScore", param.costScore);
    nhPrivate.getParam("useCost", param.useCost);
    nhPrivate.getParam("pointPerPathThre", param.pointPerPathThre);
    nhPrivate.getParam("dirWeight", param.dirWeight);
    nhPrivate.getParam("dirThre", param.dirThre);
    nhPrivate.getParam("minPathScale", param.minPathScale);
    nhPrivate.getParam("pathScaleStep", param.pathScaleStep);
    nhPrivate.getParam("pathScaleBySpeed", param.pathScaleBySpeed);
    nhPrivate.getParam("minPathRange", param.minPathRange);
    nhPrivate.getParam("pathRangeStep", param.pathRangeStep);
    nhPrivate.getParam("pathRangeBySpeed", param.pathRangeBySpeed);
    nhPrivate.getParam("pathCropByGoal", param.pathCropByGoal);
    nhPrivate.getParam("goalClearRange", param.goalClearRange);
    nhPrivate.getParam("slow_dis", param.slow_dis);
    nhPrivate.getParam("defPathScale", param.defPathScale);
    nhPrivate.getParam("usePathScale", param.usePathScale);
    nhPrivate.getParam("minSpeedRange", param.minSpeedRange);
    nhPrivate.getParam("close_map_time", param.close_map_time);
    nhPrivate.getParam("use_fail_closemap", param.use_fail_closemap);
    nhPrivate.getParam("goalClearRange_global", param.goalClearRange_global);
    nhPrivate.getParam("add_point_radius", param.add_point_radius);
    nhPrivate.getParam("vehicleLength", param.vehicleLength);
    nhPrivate.getParam("vehicleWidth", param.vehicleWidth);
}
