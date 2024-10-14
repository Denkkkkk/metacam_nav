#include "localPlanner/parameters.h"

void ParamControl::load_params()
{
    // 加载通用参数
    nhUsual.getParam("maxSpeed", param.maxSpeed);

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
    nhPrivate.getParam("use_map", param.useMap);
    nhPrivate.getParam("close_map_time", param.close_map_time);
    nhPrivate.getParam("use_fail_closemap", param.use_fail_closemap);
    nhPrivate.getParam("goalClearRange_global", param.goalClearRange_global);
    nhPrivate.getParam("add_point_radius", param.add_point_radius);
    param_origin = param;
}

void ParamControl::update_params()
{
}
