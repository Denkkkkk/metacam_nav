/**
 * @file pathFollower.cpp
 * @author 李东权
 * @brief 路径跟随和速度控制
 * @version 2.0 
 * @date 2023-11-26
 *
 * @copyright Copyright (c) 2024
 *
 */
#include "pathFollower.h"

using namespace std;

RoboCtrl::RoboCtrl()
{
    pctlPtr = new ParamControl();
    nhPrivate.getParam("vehicleX", way_point.pose.position.x);
    nhPrivate.getParam("vehicleY", way_point.pose.position.y);
    nhPrivate.getParam("ns", ns);
    goal_point_origin = way_point;
    use_real_goal = true;
    cmdTopic = ns + "/cmd_vel";
    if (ns != "")
    {
        ns += "/";
    }
    robot_frame = ns + "vehicle";
    subOdom = nh.subscribe<nav_msgs::Odometry>("/odom_interface", 5, &RoboCtrl::odomHandler, this);
    subTerrainCloud = nh.subscribe<sensor_msgs::PointCloud2>("/terrain_map", 5, &RoboCtrl::terrainCloudCallback, this);
    // 是否使用move_base全局规划
    if (pctlPtr->get_params().use_move_base)
    {
        subGlobalPoint = nh.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 5, &RoboCtrl::goalPointCallback, this);
        subPathStatus = nh.subscribe("/move_base/GlobalPlanner/goal_plan_status", 1, &RoboCtrl::pathStatusCallback, this);
        subGoalPath = nh.subscribe("/move_base/GlobalPlanner/plan", 5, &RoboCtrl::goalPathCallback, this);
        path_status.data = false;
    }
    if (pctlPtr->get_params().use_virtual_head)
    {
        pubVirHeadDir = nh.advertise<std_msgs::Float32>("/vir_head_dir", 1);
    }
    if (pctlPtr->get_params().useCloudSlowDown)
    {
        subSlowDown = nh.subscribe<std_msgs::Float32>("/slow_down", 1, &RoboCtrl::slowDownHandler, this);
    }
    subPath = nh.subscribe<nav_msgs::Path>("/local_path", 1, &RoboCtrl::pathHandler, this);
    subStop = nh.subscribe<std_msgs::Bool>("/stop", 5, &RoboCtrl::stopHandler, this);
    subGoal = nh.subscribe<geometry_msgs::PoseStamped>("/way_point", 5, &RoboCtrl::goalHandler, this);
    subControlMode = nh.subscribe("/control_mode", 5, &RoboCtrl::controlModeCallback, this);

    pubCmd_vel = nh.advertise<geometry_msgs::Twist>(cmdTopic, 5);
    pubSpeed = nh.advertise<std_msgs::Float32>("/speed", 5);
    pubGoalPathDir = nh.advertise<geometry_msgs::PoseStamped>("/goal_path_dir", 5);
    pubGetGoal = nh.advertise<std_msgs::Bool>("/get_goal", 1);
    maxSpeed1 = pctlPtr->get_params().maxSpeed;
    pub_rate = 20;
}

// 外部减速
void RoboCtrl::slowDown()
{
    double local_slowDown_update_duaration = ros::Time::now().toSec() - local_slowDown_update_time;
    if (local_slowDown_update_duaration > 2.0)
    {
        local_slowDown = 1000;
    }
    // 打印局部规划器最近的点云距离
    ROS_WARN("localplanner_cloud_minDis: %f", local_slowDown);
    // 打印terrainCloud_minDis
    ROS_WARN("terrainCloud_minDis: %f", terrainCloud_minDis);
    near_cloud_stop = false;
    if (terrainCloud_minDis < pctlPtr->get_params().vehicle_stop_range) // 急停
    {
        near_cloud_stop = true;
        maxSpeed1 = pctlPtr->get_params().cloudSlow_minSpeed;
    }
    else if (pctlPtr->get_params().localPlanner_pathRange <= 1.0) // 规划范围较小优先发起减速
    {
        maxSpeed1 = pctlPtr->get_params().maxSpeed * 0.4;
    }
    else if (terrainCloud_minDis < pctlPtr->get_params().localPlanner_slow_dis)
    {
        maxSpeed1 = pctlPtr->get_params().maxSpeed * pctlPtr->get_params().slowdown_rate;
    }

    if (maxSpeed1 < pctlPtr->get_params().cloudSlow_minSpeed)
        maxSpeed1 = pctlPtr->get_params().cloudSlow_minSpeed;
}

/**
 * @brief 正常发布速度
 * @attention cmd_vel必须无条件相应速度值，不然跟踪器无法跟踪修正，会陷入死局
 * @get_params() vehicleSpeed
 */
void RoboCtrl::pubVehicleSpeed(const double vehicleSpeed)
{
    if (pctlPtr->get_params().use_virtual_head)
    {
        double vh_to_v;
        car_speed.data = vehicleSpeed;
        vh_to_v = vehicleYaw - virture_headDir;
        cmd_vel.linear.x = vehicleSpeed * cos(vh_to_v);
        cmd_vel.linear.y = -vehicleSpeed * sin(vh_to_v);
    }
    else
    {
        cmd_vel.linear.x = vehicleSpeed;    // 前向速度
        cmd_vel.angular.z = vehicleYawRate; // 旋转速度
    }

    // 优先判断到点后的yaw角控制
    double goalyaw_diff = vehicleYaw - pctlPtr->get_params().getgoal_yaw;
    if (get_goal.data && pctlPtr->get_params().use_getgoal_yaw && abs(goalyaw_diff) > 0.1 && abs(vehicleSpeed) < 0.001)
    {
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.x = -1.0;
        // 转向控制，上下死区
        if (abs(goalyaw_diff) < 0.1)
        {
            cmd_vel.angular.z = (goalyaw_diff > 0) ? -0.1 : 0.1;
        }
        else if (abs(goalyaw_diff) > PI / 3)
        {
            cmd_vel.angular.z = (goalyaw_diff > 0) ? -PI / 4 : PI / 4;
        }
        else
        {
            cmd_vel.angular.z = -goalyaw_diff;
        }
    }
    else
    {
        static int stop_times = 0; // 在导航可控的情况下确保车辆停稳后才转遥控接管
        if (safetyStop && abs(vehicleSpeed) < 0.01 && stop_times > 10)
        {
            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.x = 0.0;
            cmd_vel.angular.z = 0.0;
        }
        else
        {
            if (stop_times <= 10 && abs(vehicleSpeed) < 0.001)
            {
                stop_times++;
            }
            else
            {
                stop_times = 0;
            }
            cmd_vel.angular.x = -1.0;
        }
    }
    pubCmd_vel.publish(cmd_vel);
    car_speed.data = vehicleSpeed;
    pubSpeed.publish(car_speed);
    ROS_WARN("vehicleSpeed: %f", vehicleSpeed);
    ROS_WARN("cmd_vel.linear.x: %f, cmd_vel.angular.z:%f", cmd_vel.linear.x, cmd_vel.angular.z);
}
/**
 * @brief 以全局目标点为方向发布速度
 *
 * @get_params() vehicleSpeed
 * @get_params() goal_dir
 */
void RoboCtrl::pubVehicleSpeed_goalDir(const double vehicleSpeed, const double goal_dir)
{
    car_speed.data = vehicleSpeed;
    if (pctlPtr->get_params().use_virtual_head)
    {
        double goal_to_vhi = vehicleYaw - goal_dir;
        cmd_vel.linear.x = vehicleSpeed * cos(goal_to_vhi);
        cmd_vel.linear.y = -vehicleSpeed * sin(goal_to_vhi);
    }
    else
    {
        double goal_to_vhi = vehicleYaw - goal_dir;
        vehicleYawRate = -pctlPtr->get_params().yawRateGain * goal_to_vhi;
        // 旋转速度过大保护
        if (vehicleYawRate > pctlPtr->get_params().maxYawRate * PI / 180.0)
            vehicleYawRate = pctlPtr->get_params().maxYawRate * PI / 180.0; // 最大最小值限制
        else if (vehicleYawRate < -pctlPtr->get_params().maxYawRate * PI / 180.0)
            vehicleYawRate = -pctlPtr->get_params().maxYawRate * PI / 180.0; // 一秒最大转45度时，对应-0.7854
        if (fabs(vehicleYawRate) > fabs(goal_to_vhi) * 3)
        {
            vehicleYawRate = -goal_to_vhi * 3;
        }
        cmd_vel.angular.z = vehicleYawRate; // 旋转速度
        cmd_vel.linear.x = vehicleSpeed;    // 前向速度

        static int stop_times = 0; // 在导航可控的情况下确保车辆停稳后才转遥控接管
        if (safetyStop && abs(vehicleSpeed) < 0.01 && stop_times > 10)
        {
            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.x = 0.0;
            cmd_vel.angular.z = 0.0;
        }
        else
        {
            if (stop_times <= 10 && abs(vehicleSpeed) < 0.001)
            {
                stop_times++;
            }
            else
            {
                stop_times = 0;
            }
            cmd_vel.angular.x = -1.0;
        }
    }
    pubCmd_vel.publish(cmd_vel);
    pubSpeed.publish(car_speed);
    ROS_WARN("vehicleSpeed: %f", vehicleSpeed);
    ROS_WARN("cmd_vel.linear.x: %f, cmd_vel.angular.z:%f", cmd_vel.linear.x, cmd_vel.angular.z);
}

void RoboCtrl::slowStop()
{
    if (vehicleSpeed < 0)
    {
        vehicleSpeed += pctlPtr->get_params().maxSlowAccel * 10 / pub_rate;
        if (vehicleSpeed > -pctlPtr->get_params().minSpeed || vehicleSpeed > -pctlPtr->get_params().maxSlowAccel * 10 / pub_rate)
        {
            vehicleSpeed = 0;
            vehicleYawRate = 0;
        }
    }
    else if (vehicleSpeed > 0)
    {
        vehicleSpeed -= pctlPtr->get_params().maxSlowAccel * 10 / pub_rate;
        if (vehicleSpeed < pctlPtr->get_params().minSpeed || vehicleSpeed < pctlPtr->get_params().maxSlowAccel * 10 / pub_rate)
        {
            vehicleSpeed = 0;
            vehicleYawRate = 0;
        }
    }

    pubVehicleSpeed(vehicleSpeed);
}

void RoboCtrl::pure_persuit()
{

    maxSpeed1 = pctlPtr->get_params().maxSpeed; // 恢复最大速度
    // 全局目标点到最新车体位置的距离
    if (pctlPtr->get_params().use_move_base)
    {
        goal_point = goal_point_origin;
    }
    else
    {
        goal_point = way_point;
    }
    float endDisX = goal_point.pose.position.x - vehicleX;
    float endDisY = goal_point.pose.position.y - vehicleY;
    endGoalDis_now = sqrt(endDisX * endDisX + endDisY * endDisY);
    // 虚拟全局目标点到最新车体位置的距离
    float virture_endDisX = virture_goalX - vehicleX;
    float virture_endDisY = virture_goalY - vehicleY;
    virture_endGoalDis_now = sqrt(virture_endDisX * virture_endDisX + virture_endDisY * virture_endDisY);

    /**
     * @brief 二次保护的到点状态判断
     *
     */
    if (!use_real_goal && virture_endGoalDis_now > pctlPtr->get_params().endGoalDis + 0.3)
    {
        use_real_goal = true;
        get_goal.data = false;
        pubGetGoal.publish(get_goal);
    }
    else if (use_real_goal && endGoalDis_now < pctlPtr->get_params().endGoalDis)
    {
        virture_goalX = vehicleX;
        virture_goalY = vehicleY;
        use_real_goal = false;
        get_goal.data = true;
        pubGetGoal.publish(get_goal);
    }
    else if (use_real_goal && endGoalDis_now >= pctlPtr->get_params().endGoalDis)
    {
        get_goal.data = false;
        pubGetGoal.publish(get_goal);
    }
    else if (!use_real_goal && virture_endGoalDis_now <= pctlPtr->get_params().endGoalDis + 0.3)
    {
        get_goal.data = true;
        pubGetGoal.publish(get_goal);
    }

    /**
     * @brief 是否直接跟随全局路径
     *
     */
    nav_msgs::Path splined_path;
    if (pctlPtr->get_params().use_move_base && path_status.data && pctlPtr->get_params().goal_path_direct && (control_mode == MIDPlanning || control_mode == GUIDEPlanning))
    {
        splined_path = goal_path;
    }
    else
    {
        splined_path = path;
    }
    if (pctlPtr->get_params().useCloudSlowDown)
    {
        slowDown();
    }
    /**
     * @brief 转弯点减速
     *
     */
    if (pctlPtr->get_params().use_move_base)
    {
        if (control_mode == MIDPlanning && pctlPtr->get_params().use_MIDPlanning_slow)
        {
            maxSpeed1 *= pctlPtr->get_params().MIDPlanning_slow_rate;
            if (maxSpeed1 < pctlPtr->get_params().MIDPlanning_minSpeed)
                maxSpeed1 = pctlPtr->get_params().MIDPlanning_minSpeed;
            mid_slow_delay = 0;
        }
        else if (mid_slow_delay < 50)
        {
            maxSpeed1 *= pctlPtr->get_params().MIDPlanning_slow_rate;
            if (maxSpeed1 < pctlPtr->get_params().MIDPlanning_minSpeed)
                maxSpeed1 = pctlPtr->get_params().MIDPlanning_minSpeed;
            mid_slow_delay++;
        }
    }
    ROS_WARN("maxSpeed1: %f", maxSpeed1);
    /**
     * @brief 距离计算
     *
     */
    int pathSize = splined_path.poses.size();
    if (pathSize >= 2 && pathInit)
    {
        // 计算最新的车体位置相对刚获取到路径信息时的位置的相对坐标
        vehicleXRel = cos(vehicleYawRec) * (vehicleX - vehicleXRec) + sin(vehicleYawRec) * (vehicleY - vehicleYRec);
        vehicleYRel = -sin(vehicleYawRec) * (vehicleX - vehicleXRec) + cos(vehicleYawRec) * (vehicleY - vehicleYRec);
        // 最优路径的最后一个点到最新车体位置的X方向的距离
        float endpathDisX = splined_path.poses[pathSize - 1].pose.position.x - vehicleXRel;
        float endpathDisY = splined_path.poses[pathSize - 1].pose.position.y - vehicleYRel;
        endPathDis_now = sqrt(endpathDisX * endpathDisX + endpathDisY * endpathDisY);
    }
    // 遍历所有的路径点，循环的目的是找到机器人需要前进到的路径点。这是路径跟踪的核心部分

    double odom_update_duration = ros::Time::now().toSec() - odom_update_time;
    bool no_odom_flag = false;
    if (odom_update_duration > 1.0) // 一秒都收不到里程计数据
    {
        no_odom_flag = true;
    }
    /**
     * @brief 到点或异常，优先判断并输出状态
     *
     */
    if (no_odom_flag || pathSize < 2 || get_goal.data || !pathInit || safetyStop || near_cloud_stop)
    {
        vehicleYawRate = 0;
        slowStop();
        if (get_goal.data)
        {
            ROS_ERROR("GetGoal STOP!");
        }
        if (no_odom_flag)
        {
            ROS_ERROR("NoOdom_get STOP!");
        }
        if (safetyStop)
        {
            // 外部请求强制停车
            pathInit = false;
            ROS_ERROR("SafetyStop STOP!");
        }
        if (pathSize < 2 || !pathInit)
        {
            ROS_ERROR("NoPath_get STOP!");
        }
        if (near_cloud_stop)
        {
            ROS_ERROR("NearCloudStop STOP!");
        }
        return;
    }
    /**
     * @brief 到目标点附近，直接冲向目标点
     *
     */
    else if (pctlPtr->get_params().use_closeGoal_direct && endGoalDis_now < pctlPtr->get_params().closeGoal_direct_dis) // 直接直线前往目标点
    {
        double goal_dir = atan2(endDisY, endDisX);
        vehicleSpeed = pctlPtr->get_params().close_direct_speed;
        pubVehicleSpeed_goalDir(vehicleSpeed, goal_dir);
        ROS_INFO("closeGoal_direct!");
        return;
    }
    else
    {
        float disX, disY, dis;
        // 每次收到/path的时候都会重置pathPointID为0，然后一直累加
        while (pathPointID < pathSize - 1)
        {
            disX = splined_path.poses[pathPointID].pose.position.x - vehicleXRel;
            disY = splined_path.poses[pathPointID].pose.position.y - vehicleYRel;
            dis = sqrt(disX * disX + disY * disY); // 计算机器人当前位置与路径上各个点之间的距离
            if (dis < pctlPtr->get_params().lookAheadDis)
            {
                pathPointID++;
            }
            else
            {
                break; // 当路径点越来越远且刚好达到阈值时，就是向量长度
            }
        }
        // 找到pathPointID，确定机器人应该朝向路径点的方向,以车头为系,为什么循环外要再算一次，因为用了局部变量，return完就没了，下次可能进不了while循环
        disX = splined_path.poses[pathPointID].pose.position.x - vehicleXRel;
        disY = splined_path.poses[pathPointID].pose.position.y - vehicleYRel;
        dis = sqrt(disX * disX + disY * disY); // 计算机器人当前位置与前向点之间的距离
        float pathDir = atan2(disY, disX);

        // 航向角,最新车头相对路径的方向差
        float dirDiff;
        if (pctlPtr->get_params().use_virtual_head)
        {
            dirDiff = virture_headDir - vehicleYawRec - pathDir;
        }
        else
        {
            dirDiff = vehicleYaw - vehicleYawRec - pathDir;
        }
        while (dirDiff > M_PI || dirDiff < -M_PI)
        {
            if (dirDiff > M_PI)
                dirDiff -= 2 * M_PI;
            else if (dirDiff < -M_PI)
                dirDiff += 2 * M_PI;
        }
        double time = ros::Time::now().toSec();
        if (use_two_forward)
        {
            // 如果机器人当前的 dirDiff 大于 π/2（90度）并且机器人正朝前行驶 (navFwd)，并且距离上一次切换方向的时间超过阈值 switchTimeThre
            if (fabs(dirDiff) > PI / 2 && navFwd && time - switchTime > switchTimeThre)
            {
                navFwd = false;
                switchTime = time;
            }
            else if (fabs(dirDiff) < PI / 2 && !navFwd && time - switchTime > switchTimeThre)
            {
                navFwd = true;
                switchTime = time;
            }
        }
        else
        {
            navFwd = true;
        }
        float joySpeed2 = maxSpeed1;
        if (!navFwd)
        { // 如果机器人不是正向前行驶
            dirDiff += PI;
            if (dirDiff > PI)
                dirDiff -= 2 * PI;
            joySpeed2 *= -1;
        }
        if (abs(dirDiff) < pctlPtr->get_params().dirDiffThre_keep)
        {
            vehicleYawRate = 0.0;
        }
        else if (fabs(vehicleSpeed) < pctlPtr->get_params().quick_turn_speed)
        {
            if (abs(dirDiff) < 0.2)
            {
                vehicleYawRate = -pctlPtr->get_params().stopYawRateGain * dirDiff / 3.0; // 偏差角较小时不用转这么快
            }
            else
            {
                vehicleYawRate = -pctlPtr->get_params().stopYawRateGain * dirDiff; // 如果机器人的速度小于一定阈值，航向角速率将根据 stopYawRateGain进行调整，否则yawRateGai
            }
            if (vehicleYawRate > pctlPtr->get_params().maxStopYawRate * PI / 180.0)
                vehicleYawRate = pctlPtr->get_params().maxStopYawRate * PI / 180.0; // 最大最小值限制
            else if (vehicleYawRate < -pctlPtr->get_params().maxStopYawRate * PI / 180.0)
                vehicleYawRate = -pctlPtr->get_params().maxStopYawRate * PI / 180.0; // 一秒最大转45度时，对应-0.7854
            // 增益过大
            if (fabs(vehicleYawRate) > fabs(dirDiff) * 5)
            {
                vehicleYawRate = -dirDiff * 5;
            }
        }
        else
        {
            if (abs(dirDiff) < 0.2)
            {
                vehicleYawRate = -pctlPtr->get_params().yawRateGain * dirDiff / 1.8; // 偏差角较小时不用转这么快
            }
            else
            {
                vehicleYawRate = -pctlPtr->get_params().yawRateGain * dirDiff; // 如果机器人的速度小于一定阈值，航向角速率将根据 stopYawRateGain进行调整，否则yawRateGai
            }
            if (vehicleYawRate > pctlPtr->get_params().maxYawRate * PI / 180.0)
                vehicleYawRate = pctlPtr->get_params().maxYawRate * PI / 180.0; // 最大最小值限制
            else if (vehicleYawRate < -pctlPtr->get_params().maxYawRate * PI / 180.0)
                vehicleYawRate = -pctlPtr->get_params().maxYawRate * PI / 180.0; // 一秒最大转45度时，对应-0.7854
            // 增益过大
            if (fabs(vehicleYawRate) > fabs(dirDiff) * 5)
            {
                vehicleYawRate = -dirDiff * 5;
            }
        }
        ROS_WARN("dirDiff: %f ;vehicleYawRate: %f", dirDiff, vehicleYawRate);

        // 追踪到路径末尾减速
        if (endPathDis_now / pctlPtr->get_params().pathSlowDisThre < 1 && pctlPtr->get_params().useLoaclSlow)
        {
            pathSlow_K = log(pctlPtr->get_params().getPath_speed / abs(joySpeed2)) / log(pctlPtr->get_params().endPathDis / pctlPtr->get_params().pathSlowDisThre); // 换底公式实现底数计算
            joySpeed2 = (joySpeed2 - pctlPtr->get_params().path_zero_bias) * (pow(endPathDis_now, pathSlow_K) / pow(pctlPtr->get_params().pathSlowDisThre, pathSlow_K)) + pctlPtr->get_params().path_zero_bias;
        }
        // 靠近全局目标点减速
        if (endGoalDis_now / pctlPtr->get_params().goalSlowDisThre < 1)
        {
            // 计算到点减速的指数值
            goalSlow_K = log(pctlPtr->get_params().getGoal_speed / abs(joySpeed2)) / log(pctlPtr->get_params().closeGoal_direct_dis / pctlPtr->get_params().goalSlowDisThre); // 换底公式实现底数计算
            joySpeed2 = (joySpeed2 - pctlPtr->get_params().goal_zero_bias) * (pow(endGoalDis_now, goalSlow_K) / pow(pctlPtr->get_params().goalSlowDisThre, goalSlow_K)) + pctlPtr->get_params().goal_zero_bias;
        }
        // 由于joySpeed2是允许正常规划下的速度计算，理论上不应该停车，也不应该太小导致失速
        if (abs(joySpeed2) < 0.08)
        {
            joySpeed2 = joySpeed2 > 0 ? 0.08 : -0.08;
        }
        ROS_WARN("joySpeed2: %f", joySpeed2);

        // 当偏差方向在直行区间，全速前进
        if (fabs(dirDiff) < pctlPtr->get_params().dirDiffThre_slow)
        {
            if (vehicleSpeed > (joySpeed2 - pctlPtr->get_params().maxAddAccel / pub_rate) ||
                vehicleSpeed < (joySpeed2 + pctlPtr->get_params().maxAddAccel / pub_rate) ||
                vehicleSpeed > (joySpeed2 - pctlPtr->get_params().maxSlowAccel / pub_rate) ||
                vehicleSpeed < (joySpeed2 + pctlPtr->get_params().maxSlowAccel / pub_rate))
            {
                vehicleSpeed = joySpeed2;
            }
            else
            {
                if (abs(vehicleSpeed) > abs(joySpeed2) || vehicleSpeed * joySpeed2 < 0)
                {
                    vehicleSpeed += vehicleSpeed > joySpeed2 ? -pctlPtr->get_params().maxSlowAccel / pub_rate : pctlPtr->get_params().maxSlowAccel / pub_rate;
                }
                else
                {
                    vehicleSpeed += vehicleSpeed > joySpeed2 ? -pctlPtr->get_params().maxAddAccel / pub_rate : pctlPtr->get_params().maxAddAccel / pub_rate;
                }
            }
            // 速度最大限制
            if (abs(vehicleSpeed) > pctlPtr->get_params().maxSpeed)
            {
                vehicleSpeed = vehicleSpeed > 0 ? pctlPtr->get_params().maxSpeed : -pctlPtr->get_params().maxSpeed;
            }

            // 速度小允许跳变
            if (fabs(vehicleSpeed) < pctlPtr->get_params().minSpeed)
            {
                if (fabs(joySpeed2) > pctlPtr->get_params().minSpeed)
                {
                    vehicleSpeed = joySpeed2 > 0 ? pctlPtr->get_params().minSpeed : -pctlPtr->get_params().minSpeed;
                }
                else
                {
                    vehicleSpeed = joySpeed2;
                }
            }
        }
        // 偏差较大减速，先转到对应方向再向前
        else
        {
            double slowAccel_rate = 1;
            if (fabs(dirDiff) > 0.28)
            {
                slowAccel_rate = 10;
            }
            if (vehicleSpeed > 0)
                vehicleSpeed -= pctlPtr->get_params().maxSlowAccel / pub_rate * slowAccel_rate;
            else if (vehicleSpeed < 0)
                vehicleSpeed += pctlPtr->get_params().maxSlowAccel / pub_rate * slowAccel_rate;
            if (abs(vehicleSpeed) < pctlPtr->get_params().maxSlowAccel / pub_rate * slowAccel_rate)
            {
                vehicleSpeed = 0;
            }
        }
        // 角速度
        if (pctlPtr->get_params().use_virtual_head)
        {
            virture_headDir = virture_headDir + vehicleYawRate;
            while (virture_headDir > M_PI || virture_headDir < -M_PI)
            {
                if (virture_headDir > M_PI)
                    virture_headDir -= 2 * M_PI;
                else if (virture_headDir < -M_PI)
                    virture_headDir += 2 * M_PI;
            }
            std_msgs::Float32 vehi_headDir;
            vehi_headDir.data = virture_headDir;
            pubVirHeadDir.publish(vehi_headDir);
        }
        pubVehicleSpeed(vehicleSpeed);

        if (pctlPtr->get_params().use_move_base && pctlPtr->get_params().goal_path_direct && path_status.data)
        {
            goal_path_dir.pose.position.x = 0;
            goal_path_dir.pose.position.y = 0;
            goal_path_dir.pose.position.z = 0;
            geometry_msgs::Quaternion geoQuat = tf::createQuaternionMsgFromRollPitchYaw(0, 0, pathDir);
            goal_path_dir.pose.orientation = geoQuat;
            goal_path_dir.header.frame_id = robot_frame;
            goal_path_dir.header.stamp = ros::Time::now();
            pubGoalPathDir.publish(goal_path_dir);
        }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pathFollower");
    RoboCtrl roboctrl;
    ros::Rate rate(roboctrl.pub_rate);
    bool status = ros::ok();
    while (status)
    {
        auto start = std::chrono::high_resolution_clock::now();
        ros::spinOnce();
        roboctrl.pctlPtr->update_params();
        roboctrl.pure_persuit();
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
        // 静态变量降低打印频率
        static int print_count = 0;
        if (print_count % 10 == 0)
        {
            ROS_INFO("run one pathFollower time: %f ms", duration.count() / 1000.0);
            print_count = 0;
        }
        print_count++;
        status = ros::ok();
        rate.sleep();
    }
    return 0;
}
