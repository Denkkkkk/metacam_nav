#include "dynamic_reconfigure/server.h"
#include "lplanner_reconfigure/lplanner_control.h"
#include "lplanner_reconfigure/lplannerdataConfig.h"
#include "ros/ros.h"

void reconfigure_callback(lplannerdata::lplannerdataConfig &config, uint32_t level)
{
    static ros::NodeHandle nh;
    static ros::Publisher lplanner_data_pub = nh.advertise<lplanner_reconfigure::lplanner_control>("/lplanner_control_data", 10);

    static lplanner_reconfigure::lplanner_control lplanner_control_data;
    lplanner_control_data.maxSpeed = config.maxSpeed;
    lplanner_control_data.vehicleRadius = config.vehicleRadius;
    lplanner_control_data.terrainVoxelSize = config.terrainVoxelSize;
    lplanner_control_data.useMap = config.useMap;
    lplanner_control_data.close_map_time = config.close_map_time;
    lplanner_control_data.adjacentRange = config.adjacentRange;
    lplanner_control_data.pathRangeStep = config.pathRangeStep;
    lplanner_control_data.minSpeedRange = config.minSpeedRange;
    lplanner_control_data.minPathRange = config.minPathRange;
    lplanner_control_data.pathScaleBySpeed = config.pathScaleBySpeed;
    lplanner_control_data.obstacleHeightThre = config.obstacleHeightThre;
    lplanner_control_data.dirWeight = config.dirWeight;
    lplanner_control_data.dirThre = config.dirThre;
    lplanner_control_data.detourWei = config.detourWei;
    lplanner_control_data.goalClearRange = config.goalClearRange;
    lplanner_control_data.usePathScale = config.usePathScale;
    lplanner_control_data.pathScale = config.pathScale;
    lplanner_control_data.minPathScale = config.minPathScale;
    lplanner_control_data.pathScaleStep = config.pathScaleStep;

    ROS_WARN("参数已更新，当前参数为:\n"
             "1. 最大速度：%f\n"
             "2. 车半径：%f\n"
             "3. 下采样体素边长：%f\n"
             "4. 是否使用地图：%d\n"
             "5. 单次关闭局部点云图的时长：%f\n"
             "6. 第一次裁剪规划点云的范围：%f\n"
             "7. 规划范围缩小步长：%f\n"
             "8. 速度可控最小规划范围：%f\n"
             "9. 最小规划范围：%f\n"
             "10. 是否根据速度调整缩放：%d\n"
             "11. 障碍物认定阈值：%f\n"
             "12. 只终点在此转弯角内的路径：%f\n"
             "13. 只在这扇形规划：%f\n"
             "14. 绕行系数：%f\n"
             "15. 目标点规划范围限制：%f\n"
             "16. 使用地图缩放：%d\n"
             "17. 缩放系数：%f\n"
             "18. 最小缩放系数：%f\n"
             "19. 缩放系数减小步长：%f\n",
             config.maxSpeed,
             config.vehicleRadius,
             config.terrainVoxelSize,
             config.useMap,
             config.close_map_time,
             config.adjacentRange,
             config.pathRangeStep,
             config.minSpeedRange,
             config.minPathRange,
             config.pathScaleBySpeed,
             config.obstacleHeightThre,
             config.dirWeight,
             config.dirThre,
             config.detourWei,
             config.goalClearRange,
             config.usePathScale,
             config.pathScale,
             config.minPathScale,
             config.pathScaleStep);

    ROS_WARN("改变的参数编号：%d\n", level);
    lplanner_data_pub.publish(lplanner_control_data);
}

int main(int argc, char *argv[])
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "lplannerdata_config");
    // 创建动态调参服务器，接收到参数改变的请求会执行回调
    dynamic_reconfigure::Server<lplannerdata::lplannerdataConfig> dynamic_reconfigure_server;
    auto callback = boost::bind(&reconfigure_callback, _1, _2);
    dynamic_reconfigure_server.setCallback(callback);
    ROS_WARN("spinningnode \n");
    ros::spin();
    return 0;
}