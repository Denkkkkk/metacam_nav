#include "pathFollower/parameters.h"

void ParamControl::load_params()
{
    nhPrivate.getParam("lookAheadDis", param.lookAheadDis);
    nhPrivate.getParam("maxSpeed", param.maxSpeed);
    nhPrivate.getParam("pathSlowDisThre", param.pathSlowDisThre);
    nhPrivate.getParam("spinSpeed", param.spinSpeed);
    nhPrivate.getParam("endGoalDis", param.endGoalDis);
    nhPrivate.getParam("endPathDis", param.endPathDis);
    nhPrivate.getParam("useCloudSlowDown", param.useCloudSlowDown);
    nhPrivate.getParam("cloudSlow_minSpeed", param.cloudSlow_minSpeed);
    nhPrivate.getParam("minSpeed", param.minSpeed);
    nhPrivate.getParam("curvature", param.curvature);
    nhPrivate.getParam("slowBegin", param.slowBegin);
    nhPrivate.getParam("safetyStop", param.safetyStop);
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
    nhPrivate.getParam("dirDiffThre", param.dirDiffThre);
    nhPrivate.getParam("getPath_speed", param.getPath_speed);
    nhPrivate.getParam("path_zero_bias", param.path_zero_bias);
    nhPrivate.getParam("goal_zero_bias", param.goal_zero_bias);
    nhPrivate.getParam("MIDPlanning_minSpeed", param.MIDPlanning_minSpeed);
    nhPrivate.getParam("use_closeGoal_direct", param.use_closeGoal_direct);
    nhPrivate.getParam("closeGoal_direct_dis", param.closeGoal_direct_dis);
    nhPrivate.getParam("quick_turn_N", param.quick_turn_N);
    nhPrivate.param("close_direct_speed", param.close_direct_speed, 0.4);
    nhPrivate.param("update_begin_dis_goalPoint", param.update_begin_dis_goalPoint, 4.0);
    nhPrivate.param("update_begin_dis_vehicleDis", param.update_begin_dis_vehicleDis, 5.0);
    nhPrivate.param("use_virtual_head", param.use_virtual_head, false);
    nhPrivate.param("use_move_base", param.use_move_base, false);
    param_origin = param;
}

void ParamControl::update_params()
{
    // 允许直接更新的参数
    nhPrivate.getParam("MIDPlanning_slow_rate", param.MIDPlanning_slow_rate);
}

void ParamControl::output_params()
{
    nhPrivate_actual.setParam("lookAheadDis", param.lookAheadDis);
    nhPrivate_actual.setParam("maxSpeed", param.maxSpeed);
    nhPrivate_actual.setParam("pathSlowDisThre", param.pathSlowDisThre);
    nhPrivate_actual.setParam("spinSpeed", param.spinSpeed);
    nhPrivate_actual.setParam("endGoalDis", param.endGoalDis);
    nhPrivate_actual.setParam("endPathDis", param.endPathDis);
    nhPrivate_actual.setParam("useCloudSlowDown", param.useCloudSlowDown);
    nhPrivate_actual.setParam("cloudSlow_minSpeed", param.cloudSlow_minSpeed);
    nhPrivate_actual.setParam("minSpeed", param.minSpeed);
    nhPrivate_actual.setParam("curvature", param.curvature);
    nhPrivate_actual.setParam("slowBegin", param.slowBegin);
    nhPrivate_actual.setParam("safetyStop", param.safetyStop);
    nhPrivate_actual.setParam("maxAddAccel", param.maxAddAccel);
    nhPrivate_actual.setParam("maxSlowAccel", param.maxSlowAccel);
    nhPrivate_actual.setParam("stopYawRateGain", param.stopYawRateGain);
    nhPrivate_actual.setParam("yawRateGain", param.yawRateGain);
    nhPrivate_actual.setParam("maxYawRate", param.maxYawRate);
    nhPrivate_actual.setParam("maxStopYawRate", param.maxStopYawRate);
    nhPrivate_actual.setParam("goal_path_direct", param.goal_path_direct);
    nhPrivate_actual.setParam("useLoaclSlow", param.useLoaclSlow);
    nhPrivate_actual.setParam("goalSlowDisThre", param.goalSlowDisThre);
    nhPrivate_actual.setParam("getGoal_speed", param.getGoal_speed);
    nhPrivate_actual.setParam("use_MIDPlanning_slow", param.use_MIDPlanning_slow);
    nhPrivate_actual.setParam("MIDPlanning_slow_rate", param.MIDPlanning_slow_rate);
    nhPrivate_actual.setParam("dirDiffThre", param.dirDiffThre);
    nhPrivate_actual.setParam("getPath_speed", param.getPath_speed);
    nhPrivate_actual.setParam("path_zero_bias", param.path_zero_bias);
    nhPrivate_actual.setParam("goal_zero_bias", param.goal_zero_bias);
    nhPrivate_actual.setParam("MIDPlanning_minSpeed", param.MIDPlanning_minSpeed);
    nhPrivate_actual.setParam("use_closeGoal_direct", param.use_closeGoal_direct);
    nhPrivate_actual.setParam("closeGoal_direct_dis", param.closeGoal_direct_dis);
    nhPrivate_actual.setParam("quick_turn_N", param.quick_turn_N);
    nhPrivate_actual.setParam("close_direct_speed", param.close_direct_speed);
}
