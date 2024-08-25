#include "dynamic_reconfigure/server.h"
#include "pfollower_reconfigure/pfollower_control.h"
#include "pfollower_reconfigure/pfollowerdataConfig.h"
#include "ros/ros.h"

void reconfigure_callback(pfollowerdata::pfollowerdataConfig &config, uint32_t level)
{
    static ros::NodeHandle nh;
    static ros::Publisher pfollower_data_pub = nh.advertise<pfollower_reconfigure::pfollower_control>("/pfollower_control_data", 10);

    static pfollower_reconfigure::pfollower_control pfollower_control_data;
    pfollower_control_data.maxSpeed = config.maxSpeed;
    pfollower_control_data.useLoaclSlow = config.useLoaclSlow;
    pfollower_control_data.endPathDis = config.endPathDis;
    pfollower_control_data.pathSlowDisThre = config.pathSlowDisThre;
    pfollower_control_data.getPath_speed = config.getPath_speed;
    pfollower_control_data.path_zero_bias = config.path_zero_bias;
    pfollower_control_data.goalSlowDisThre = config.goalSlowDisThre;
    pfollower_control_data.getGoal_speed = config.getGoal_speed;
    pfollower_control_data.goal_zero_bias = config.goal_zero_bias;
    pfollower_control_data.spinSpeed = config.spinSpeed;
    pfollower_control_data.useCloudSlowDown = config.useCloudSlowDown;
    pfollower_control_data.minSpeed = config.minSpeed;
    pfollower_control_data.curvature = config.curvature;
    pfollower_control_data.slowBegin = config.slowBegin;
    pfollower_control_data.safetyStop = config.safetyStop;
    pfollower_control_data.maxAddAccel = config.maxAddAccel;
    pfollower_control_data.maxSlowAccel = config.maxSlowAccel;
    pfollower_control_data.yawRateGain = config.yawRateGain;
    pfollower_control_data.stopYawRateGain = config.stopYawRateGain;
    pfollower_control_data.maxYawRate = config.maxYawRate;
    pfollower_control_data.maxStopYawRate = config.maxStopYawRate;
    pfollower_control_data.goal_path_direct = config.goal_path_direct;
    pfollower_control_data.use_MIDPlanning_slow = config.use_MIDPlanning_slow;
    pfollower_control_data.MIDPlanning_slow_rate = config.MIDPlanning_slow_rate;
    pfollower_control_data.MIDPlanning_minSpeed = config.MIDPlanning_minSpeed;
    pfollower_control_data.dirDiffThre = config.dirDiffThre;
    pfollower_control_data.use_closeGoal_direct = config.use_closeGoal_direct;
    pfollower_control_data.closeGoal_direct_dis = config.closeGoal_direct_dis;
    pfollower_control_data.quick_turn_N = config.quick_turn_N;

    ROS_WARN("参数已更新，当前参数为:\n"
             "1. 最大速度：%f\n"
             "2. 启用局部路径跟踪减速：%d\n"
             "3. 跟踪到局部路径的末尾多少距离内就不再跟踪：%f\n"
             "4. 局部路径跟踪开始减速的距离阈值：%f\n"
             "5. 追踪到局部路径末端时的速度：%f\n"
             "6. 追踪路径减速的零点偏移：%f\n"
             "7. 靠近全局目标点时减速：%f\n"
             "8. 刚好追踪到目标点时的速度：%f\n"
             "9. 到点减速的零点偏移：%f\n"
             "10. 小陀螺转速：%f\n"
             "11. 使能障碍物减速：%d\n"
             "12. 障碍物最小减速值：%f\n"
             "13. 减速曲线曲率5-15：%f\n"
             "14. 开始减速的障碍方向数：%f\n"
             "15 紧急刹车：%d\n"
             "16. 最大加速速度：%f\n"
             "17. 最大减速加速度：%f\n"
             "18. 一般情况下的伪车头转动增益：%f\n"
             "19. 静止下的伪车头转动增益：%f\n"
             "20. 伪车头最大转动角速度：%f\n"
             "21. 静止下伪车头最大转动角速度：%f\n"
             "22. 是否全局路径直接追踪：%d\n"
             "23. 是否使用转弯点减速：%d\n"
             "24. 转弯点减速比例：%f\n"
             "25 最小转弯速度：%f\n"
             "26 角度差：%f\n"
             "27. 开启到目标点附近直接冲向目标点：%d\n"
             "28. 到目标点附近的阈值：%f\n"
             "29. 减速到这个*maxAddAccel/100就允许快速转向：%f\n",
             config.maxSpeed,
             config.useLoaclSlow,
             config.endPathDis,
             config.pathSlowDisThre,
             config.getPath_speed,
             config.path_zero_bias,
             config.goalSlowDisThre,
             config.getGoal_speed,
             config.goal_zero_bias,
             config.spinSpeed,
             config.useCloudSlowDown,
             config.minSpeed,
             config.curvature,
             config.slowBegin,
             config.safetyStop,
             config.maxAddAccel,
             config.maxSlowAccel,
             config.yawRateGain,
             config.stopYawRateGain,
             config.maxYawRate,
             config.maxStopYawRate,
             config.goal_path_direct,
             config.use_MIDPlanning_slow,
             config.MIDPlanning_slow_rate,
             config.MIDPlanning_minSpeed,
             config.dirDiffThre,
             config.use_closeGoal_direct,
             config.closeGoal_direct_dis,
             config.quick_turn_N);

    ROS_WARN("改变的参数编号：%d\n", level);
    pfollower_data_pub.publish(pfollower_control_data);
}

int main(int argc, char *argv[])
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "pfollowerdata_config");
    // 创建动态调参服务器，接收到参数改变的请求会执行回调
    dynamic_reconfigure::Server<pfollowerdata::pfollowerdataConfig> dynamic_reconfigure_server;
    auto callback = boost::bind(&reconfigure_callback, _1, _2);
    dynamic_reconfigure_server.setCallback(callback);
    ROS_WARN("spinningnode \n");
    ros::spin();
    return 0;
}