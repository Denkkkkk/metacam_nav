#include "pathFollower/parameters.h"

void ParamControl::load_params()
{
    nhUsual.getParam("maxSpeed", param.maxSpeed);

    nhlocalPlanner.param("pathRange", param.localPlanner_pathRange, 3.0);

    nhPrivate.getParam("lookAheadDis", param.lookAheadDis);
    nhPrivate.getParam("pathSlowDisThre", param.pathSlowDisThre);
    nhPrivate.getParam("endGoalDis", param.endGoalDis);
    nhPrivate.getParam("endPathDis", param.endPathDis);
    nhPrivate.getParam("useCloudSlowDown", param.useCloudSlowDown);
    nhPrivate.getParam("cloudSlow_minSpeed", param.cloudSlow_minSpeed);
    nhPrivate.getParam("minSpeed", param.minSpeed);
    nhPrivate.getParam("curvature", param.curvature);
    nhPrivate.getParam("slowBegin", param.slowBegin);
    nhPrivate.getParam("maxAddAccel", param.maxAddAccel);
    nhPrivate.getParam("maxSlowAccel", param.maxSlowAccel);
    nhPrivate.getParam("stopYawRateGain", param.stopYawRateGain);
    nhPrivate.getParam("yawRateGain", param.yawRateGain);
    nhPrivate.getParam("maxYawRate", param.maxYawRate);
    nhPrivate.getParam("maxStopYawRate", param.maxStopYawRate);
    nhPrivate.getParam("goal_path_direct", param.goal_path_direct);
    nhPrivate.getParam("useLoaclSlow", param.useLoaclSlow);
    nhPrivate.getParam("goalSlowDisThre", param.goalSlowDisThre);
    nhPrivate.getParam("getGoal_speed", param.getGoal_speed);
    nhPrivate.getParam("use_MIDPlanning_slow", param.use_MIDPlanning_slow);
    nhPrivate.getParam("MIDPlanning_slow_rate", param.MIDPlanning_slow_rate);
    nhPrivate.getParam("dirDiffThre_slow", param.dirDiffThre_slow);
    nhPrivate.getParam("dirDiffThre_keep", param.dirDiffThre_keep);
    nhPrivate.getParam("getPath_speed", param.getPath_speed);
    nhPrivate.getParam("path_zero_bias", param.path_zero_bias);
    nhPrivate.getParam("goal_zero_bias", param.goal_zero_bias);
    nhPrivate.getParam("MIDPlanning_minSpeed", param.MIDPlanning_minSpeed);
    nhPrivate.getParam("use_closeGoal_direct", param.use_closeGoal_direct);
    nhPrivate.getParam("closeGoal_direct_dis", param.closeGoal_direct_dis);
    nhPrivate.getParam("quick_turn_speed", param.quick_turn_speed);
    nhPrivate.param("close_direct_speed", param.close_direct_speed, 0.4);
    nhPrivate.param("use_virtual_head", param.use_virtual_head, false);
    nhPrivate.param("use_move_base", param.use_move_base, false);
    nhPrivate.param("use_getgoal_yaw", param.use_getgoal_yaw, false);
    nhPrivate.param("getgoal_yaw", param.getgoal_yaw, 0.0);
    nhPrivate.param("slowdown_rate", param.slowdown_rate, 0.7);
    param_origin = param;
}

void ParamControl::update_params()
{
    // 允许直接更新的参数
    nhUsual.getParam("maxSpeed", param.maxSpeed);
    nhlocalPlanner.getParam("pathRange", param.localPlanner_pathRange);

    nhPrivate.getParam("lookAheadDis", param.lookAheadDis);
    nhPrivate.getParam("pathSlowDisThre", param.pathSlowDisThre);
    nhPrivate.getParam("endGoalDis", param.endGoalDis);
    nhPrivate.getParam("endPathDis", param.endPathDis);
    nhPrivate.getParam("useCloudSlowDown", param.useCloudSlowDown);
    nhPrivate.getParam("cloudSlow_minSpeed", param.cloudSlow_minSpeed);
    nhPrivate.getParam("minSpeed", param.minSpeed);
    nhPrivate.getParam("curvature", param.curvature);
    nhPrivate.getParam("slowBegin", param.slowBegin);
    nhPrivate.getParam("maxAddAccel", param.maxAddAccel);
    nhPrivate.getParam("maxSlowAccel", param.maxSlowAccel);
    nhPrivate.getParam("stopYawRateGain", param.stopYawRateGain);
    nhPrivate.getParam("yawRateGain", param.yawRateGain);
    nhPrivate.getParam("maxYawRate", param.maxYawRate);
    nhPrivate.getParam("maxStopYawRate", param.maxStopYawRate);
    nhPrivate.getParam("goal_path_direct", param.goal_path_direct);
    nhPrivate.getParam("useLoaclSlow", param.useLoaclSlow);
    nhPrivate.getParam("goalSlowDisThre", param.goalSlowDisThre);
    nhPrivate.getParam("getGoal_speed", param.getGoal_speed);
    nhPrivate.getParam("use_MIDPlanning_slow", param.use_MIDPlanning_slow);
    nhPrivate.getParam("MIDPlanning_slow_rate", param.MIDPlanning_slow_rate);
    nhPrivate.getParam("dirDiffThre_slow", param.dirDiffThre_slow);
    nhPrivate.getParam("dirDiffThre_keep", param.dirDiffThre_keep);
    nhPrivate.getParam("getPath_speed", param.getPath_speed);
    nhPrivate.getParam("path_zero_bias", param.path_zero_bias);
    nhPrivate.getParam("goal_zero_bias", param.goal_zero_bias);
    nhPrivate.getParam("MIDPlanning_minSpeed", param.MIDPlanning_minSpeed);
    nhPrivate.getParam("use_closeGoal_direct", param.use_closeGoal_direct);
    nhPrivate.getParam("closeGoal_direct_dis", param.closeGoal_direct_dis);
    nhPrivate.getParam("quick_turn_speed", param.quick_turn_speed);
    nhPrivate.getParam("close_direct_speed", param.close_direct_speed);
    nhPrivate.getParam("use_virtual_head", param.use_virtual_head);
    nhPrivate.getParam("use_move_base", param.use_move_base);
    nhPrivate.getParam("use_getgoal_yaw", param.use_getgoal_yaw);
    nhPrivate.getParam("getgoal_yaw", param.getgoal_yaw);
    nhPrivate.getParam("slowdown_rate", param.slowdown_rate);
}