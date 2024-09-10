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
    goal_point_origin = way_point;
    pctlPtr->load_params();
    use_real_goal = true;
    nhPrivate.param<std::string>("cmdTopic", cmdTopic, "/cmd_vel");
    nhPrivate.param<std::string>("ns", ns, "");
    robot_frame = ns + "/vehicle";
    subOdom = nh.subscribe<nav_msgs::Odometry>("/Odometry", 5, &RoboCtrl::odomHandler, this);
    if (pctlPtr->param.use_move_base)
    {
        subGlobalPoint = nh.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 5, &RoboCtrl::goalPointCallback, this);
        subPathStatus = nh.subscribe("/move_base/GlobalPlanner/goal_plan_status", 1, &RoboCtrl::pathStatusCallback, this);
        subGoalPath = nh.subscribe("/move_base/GlobalPlanner/plan", 5, &RoboCtrl::goalPathCallback, this);
        path_status.data = false;
    }
    if (pctlPtr->param.use_virtual_head)
    {
        pubVirHeadDir = nh.advertise<std_msgs::Float32>("/vir_head_dir", 1);
    }
    subPath = nh.subscribe<nav_msgs::Path>("/local_path", 1, &RoboCtrl::pathHandler, this);
    subStop = nh.subscribe<std_msgs::Bool>("/stop", 5, &RoboCtrl::stopHandler, this);
    subGoal = nh.subscribe<geometry_msgs::PoseStamped>("/way_point", 5, &RoboCtrl::goalHandler, this);
    subControlMode = nh.subscribe("/control_mode", 5, &RoboCtrl::controlModeCallback, this);
    // subIMU = nh.subscribe<sensor_msgs::Imu>("/livox/imu_192_168_1_100", 5, &RoboCtrl::imuCallback, this);

    pubCmd_vel = nh.advertise<geometry_msgs::Twist>(cmdTopic, 5);
    pubSpeed = nh.advertise<std_msgs::Float32>("/speed", 5);
    pubGoalPathDir = nh.advertise<geometry_msgs::PoseStamped>("/goal_path_dir", 5);
    pubGetGoal = nh.advertise<std_msgs::Bool>("/get_goal", 1);
    maxSpeed1 = pctlPtr->param.maxSpeed;
    pub_rate = 12;
}

/**
 * @brief 正常发布速度
 *
 * @param vehicleSpeed
 */
void RoboCtrl::pubVehicleSpeed(const double vehicleSpeed)
{
    if (pctlPtr->param.use_virtual_head)
    {
        // 速度太小直接赋为0
        double vh_to_v;
        car_speed.data = vehicleSpeed;
        if (fabs(vehicleSpeed) < pctlPtr->param.maxSlowAccel / pub_rate)
        {
            cmd_vel.linear.x = 0;
            cmd_vel.linear.y = 0;
        }
        else
        {
            vh_to_v = vehicleYaw - virture_headDir;
            cmd_vel.linear.x = vehicleSpeed * cos(vh_to_v);
            cmd_vel.linear.y = -vehicleSpeed * sin(vh_to_v);
        }
        if (pctlPtr->param.spinSpeed != 0) // 仿真给决策控制的云台自转
        {
            cmd_vel.angular.z = pctlPtr->param.spinSpeed;
        }
        else
        {
            cmd_vel.angular.z = 0;
        }
    }
    else
    {
        if (fabs(vehicleSpeed) <= pctlPtr->param.maxAddAccel / pub_rate)
        {
            cmd_vel.linear.x = 0; // 速度太小直接赋为0
        }
        else
        {
            cmd_vel.linear.x = vehicleSpeed; // 前向速度
        }
        cmd_vel.angular.z = vehicleYawRate; // 旋转速度
    }
    if (pctlPtr->param.safetyStop)
    {
        cmd_vel.angular.x = 0.0;
    }
    else
    {
        cmd_vel.angular.x = -1.0;
    }
    // ROS_INFO("TEST:%f, vehicleYaw: %f, pctlPtr->param.spinSpeed:%f", vh_to_v, vehicleYaw, pctlPtr->param.spinSpeed);
    pubCmd_vel.publish(cmd_vel);
    pubSpeed.publish(car_speed);
    ROS_WARN("vehicleSpeed: %f, cmd_vel.angular.z:%f", vehicleSpeed, cmd_vel.angular.z);
}
/**
 * @brief 以全局目标点为方向发布速度
 *
 * @param vehicleSpeed
 * @param goal_dir
 */
void RoboCtrl::pubVehicleSpeed_goalDir(const double vehicleSpeed, const double goal_dir)
{
    car_speed.data = vehicleSpeed;
    if (pctlPtr->param.use_virtual_head)
    {
        if (fabs(vehicleSpeed) < pctlPtr->param.maxSlowAccel / pub_rate)
        {
            cmd_vel.linear.x = 0;
            cmd_vel.linear.y = 0;
        }
        else
        {
            double goal_to_vhi = vehicleYaw - goal_dir;
            cmd_vel.linear.x = vehicleSpeed * cos(goal_to_vhi);
            cmd_vel.linear.y = -vehicleSpeed * sin(goal_to_vhi);
        }
        if (pctlPtr->param.spinSpeed != 0) // 仿真给决策控制的云台自转
        {
            cmd_vel.angular.z = pctlPtr->param.spinSpeed;
        }
        else
        {
            cmd_vel.angular.z = 0;
        }
    }
    else
    {
        double goal_to_vhi = vehicleYaw - goal_dir;
        vehicleYawRate = -pctlPtr->param.stopYawRateGain * goal_to_vhi;
        cmd_vel.linear.x = vehicleSpeed;    // 前向速度
        cmd_vel.angular.z = vehicleYawRate; // 旋转速度
    }
    pubCmd_vel.publish(cmd_vel);
    pubSpeed.publish(car_speed);
    ROS_WARN("vehicleSpeed: %f, cmd_vel.angular.z:%f", vehicleSpeed, cmd_vel.angular.z);
}

void RoboCtrl::slowStop()
{
    if (vehicleSpeed < 0)
    {
        vehicleSpeed += pctlPtr->param.maxAddAccel / pub_rate;
        if (vehicleSpeed >= -pctlPtr->param.minSpeed)
        {
            vehicleSpeed = 0;
            vehicleYawRate = 0;
        }
    }
    else if (vehicleSpeed > 0)
    {
        vehicleSpeed -= pctlPtr->param.maxSlowAccel / pub_rate;
        if (vehicleSpeed <= pctlPtr->param.minSpeed)
        {
            vehicleSpeed = 0;
            vehicleYawRate = 0;
        }
    }

    pubVehicleSpeed(vehicleSpeed);
}

void RoboCtrl::pure_persuit()
{
    // 全局目标点到最新车体位置的距离
    if (pctlPtr->param.use_move_base)
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
    if (!use_real_goal && virture_endGoalDis_now > pctlPtr->param.endGoalDis)
    {
        use_real_goal = true;
        get_goal.data = false;
        pubGetGoal.publish(get_goal);
    }
    else if (use_real_goal && endGoalDis_now < pctlPtr->param.endGoalDis)
    {
        virture_goalX = vehicleX;
        virture_goalY = vehicleY;
        use_real_goal = false;
        get_goal.data = true;
        pubGetGoal.publish(get_goal);
    }
    else if (use_real_goal && endGoalDis_now >= pctlPtr->param.endGoalDis)
    {
        get_goal.data = false;
        pubGetGoal.publish(get_goal);
    }
    else if (!use_real_goal && virture_endGoalDis_now <= pctlPtr->param.endGoalDis)
    {
        get_goal.data = true;
        pubGetGoal.publish(get_goal);
    }

    /**
     * @brief 是否直接跟随全局路径
     *
     */
    nav_msgs::Path splined_path;
    if (pctlPtr->param.use_move_base && path_status.data && pctlPtr->param.goal_path_direct && (control_mode == MIDPlanning || control_mode == GUIDEPlanning))
    {
        splined_path = goal_path;
    }
    else
    {
        splined_path = path;
    }
    /**
     * @brief 转弯点减速
     *
     */
    if (pctlPtr->param.use_move_base)
    {
        if (control_mode == MIDPlanning && pctlPtr->param.use_MIDPlanning_slow)
        {
            maxSpeed1 *= pctlPtr->param.MIDPlanning_slow_rate;
            if (maxSpeed1 < pctlPtr->param.MIDPlanning_minSpeed)
                maxSpeed1 = pctlPtr->param.MIDPlanning_minSpeed;
            mid_slow_delay = 0;
        }
        else if (mid_slow_delay < 50)
        {
            maxSpeed1 *= pctlPtr->param.MIDPlanning_slow_rate;
            if (maxSpeed1 < pctlPtr->param.MIDPlanning_minSpeed)
                maxSpeed1 = pctlPtr->param.MIDPlanning_minSpeed;
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
    // ROS_WARN("pathInit: %d", pathInit);

    double odom_update_duration = ros::Time::now().toSec() - odom_update_time;
    bool no_odom_flag = false;
    if (odom_update_duration > 1.0) // 一秒都收不到里程计数据
    {
        no_odom_flag = true;
    }
    /**
     * @brief 不管三七二十一，直接减速停车
     *
     */
    if (pctlPtr->param.safetyStop)
    {
        vehicleYawRate = 0;
        slowStop();
        pathInit = false;
        ROS_INFO("SafetyStop!");
        return;
    }
    /**
     * @brief 到点或异常
     *
     */
    else if (no_odom_flag || pathSize < 2 || get_goal.data || !pathInit)
    {
        vehicleYawRate = 0;
        slowStop();
        if (get_goal.data)
        {
            ROS_INFO("GetGoal!");
        }
        else if (no_odom_flag)
        {
            ROS_INFO("NoOdom_get!");
        }
        else
        {
            ROS_INFO("NoPath_get!");
        }
        return;
    }
    /**
     * @brief 到目标点附近，直接冲向目标点
     *
     */
    else if (pctlPtr->param.use_closeGoal_direct && endGoalDis_now < pctlPtr->param.closeGoal_direct_dis) // 直接直线前往目标点
    {
        double goal_dir = atan2(endDisY, endDisX);
        vehicleSpeed = pctlPtr->param.close_direct_speed;
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
            if (dis < pctlPtr->param.lookAheadDis)
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
        if (pctlPtr->param.use_virtual_head)
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
        ROS_WARN("dirDiff: %f", dirDiff);
        if (abs(dirDiff) < 0.05)
        {
            vehicleYawRate = 0.0;
        }
        else if (fabs(vehicleSpeed) < pctlPtr->param.quick_turn_N * pctlPtr->param.maxSlowAccel / pub_rate)
        {
            if (abs(dirDiff) < PI / 4)
            {
                vehicleYawRate = -pctlPtr->param.stopYawRateGain * dirDiff / 2.0; // 偏差角较小时不用转这么快
            }
            else
            {
                vehicleYawRate = -pctlPtr->param.stopYawRateGain * dirDiff; // 如果机器人的速度小于一定阈值，航向角速率将根据 stopYawRateGain进行调整，否则yawRateGai
            }
            if (vehicleYawRate > pctlPtr->param.maxStopYawRate * PI / 180.0)
                vehicleYawRate = pctlPtr->param.maxStopYawRate * PI / 180.0; // 最大最小值限制
            else if (vehicleYawRate < -pctlPtr->param.maxStopYawRate * PI / 180.0)
                vehicleYawRate = -pctlPtr->param.maxStopYawRate * PI / 180.0; // 一秒最大转45度时，对应-0.7854
            // 增益过大
            if (fabs(vehicleYawRate / pub_rate) > fabs(dirDiff))
            {
                vehicleYawRate = -dirDiff * pub_rate;
            }
        }
        else
        {
            if (abs(dirDiff) < PI / 7) // 25.5度误差内
            {
                vehicleYawRate = -pctlPtr->param.yawRateGain * dirDiff / 2.0; // 偏差角较小时不用转这么快
            }
            else
            {
                vehicleYawRate = -pctlPtr->param.yawRateGain * dirDiff; // 如果机器人的速度小于一定阈值，航向角速率将根据 stopYawRateGain进行调整，否则yawRateGai
            }
            if (vehicleYawRate > pctlPtr->param.maxYawRate * PI / 180.0)
                vehicleYawRate = pctlPtr->param.maxYawRate * PI / 180.0; // 最大最小值限制
            else if (vehicleYawRate < -pctlPtr->param.maxYawRate * PI / 180.0)
                vehicleYawRate = -pctlPtr->param.maxYawRate * PI / 180.0; // 一秒最大转45度时，对应-0.7854
            // 增益过大
            if (fabs(vehicleYawRate / pub_rate) > fabs(dirDiff))
            {
                vehicleYawRate = -dirDiff * pub_rate;
            }
        }
        ROS_WARN("vehicleYawRate: %f", vehicleYawRate);

        // 追踪到路径末尾减速
        if (endPathDis_now / pctlPtr->param.pathSlowDisThre < 1 && pctlPtr->param.useLoaclSlow)
        {
            pathSlow_K = log(pctlPtr->param.getPath_speed / abs(joySpeed2)) / log(pctlPtr->param.endPathDis / pctlPtr->param.pathSlowDisThre); // 换底公式实现底数计算
            joySpeed2 = (joySpeed2 - pctlPtr->param.path_zero_bias) * (pow(endPathDis_now, pathSlow_K) / pow(pctlPtr->param.pathSlowDisThre, pathSlow_K)) + pctlPtr->param.path_zero_bias;
        }
        // 靠近全局目标点减速
        if (endGoalDis_now / pctlPtr->param.goalSlowDisThre < 1)
        {
            // 计算到点减速的指数值
            goalSlow_K = log(pctlPtr->param.getGoal_speed / abs(joySpeed2)) / log(pctlPtr->param.closeGoal_direct_dis / pctlPtr->param.goalSlowDisThre); // 换底公式实现底数计算
            joySpeed2 = (joySpeed2 - pctlPtr->param.goal_zero_bias) * (pow(endGoalDis_now, goalSlow_K) / pow(pctlPtr->param.goalSlowDisThre, goalSlow_K)) + pctlPtr->param.goal_zero_bias;
        }
        ROS_WARN("joySpeed2: %f", joySpeed2);

        // 当偏差方向在直行区间，全速前进
        // ROS_ERROR("dirDiff: %f", dirDiff);
        if (fabs(dirDiff) < pctlPtr->param.dirDiffThre)
        {
            if (vehicleSpeed < joySpeed2)
                vehicleSpeed += pctlPtr->param.maxAddAccel / pub_rate;
            else if (vehicleSpeed > joySpeed2)
                vehicleSpeed -= pctlPtr->param.maxSlowAccel / pub_rate;

            // 速度小允许跳变
            if (fabs(vehicleSpeed) < pctlPtr->param.minSpeed)
            {
                if (joySpeed2 > 0 && fabs(joySpeed2) > pctlPtr->param.minSpeed)
                {
                    vehicleSpeed = pctlPtr->param.minSpeed;
                }
                else if (joySpeed2 < 0 && fabs(joySpeed2) > pctlPtr->param.minSpeed)
                {
                    vehicleSpeed = -pctlPtr->param.minSpeed;
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
            if (vehicleSpeed > (pctlPtr->param.quick_turn_N * pctlPtr->param.maxSlowAccel / pub_rate))
                vehicleSpeed -= pctlPtr->param.maxSlowAccel / pub_rate;
            else if (vehicleSpeed < -(pctlPtr->param.quick_turn_N * pctlPtr->param.maxSlowAccel / pub_rate))
                vehicleSpeed += pctlPtr->param.maxAddAccel / pub_rate;
        }
        // 角速度
        if (pctlPtr->param.use_virtual_head)
        {
            virture_headDir = virture_headDir + vehicleYawRate / pub_rate;
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

        if (pctlPtr->param.use_move_base && pctlPtr->param.goal_path_direct && path_status.data)
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
    maxSpeed1 = pctlPtr->param.maxSpeed; // 恢复最大速度
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pathFollower");
    RoboCtrl roboctrl;
    ros::Rate rate(roboctrl.pub_rate);
    bool status = ros::ok();
    while (status)
    {
        ros::spinOnce();
        roboctrl.pctlPtr->update_params();
        roboctrl.pure_persuit();
        roboctrl.pctlPtr->output_params();
        status = ros::ok();
        rate.sleep();
    }
    return 0;
}
