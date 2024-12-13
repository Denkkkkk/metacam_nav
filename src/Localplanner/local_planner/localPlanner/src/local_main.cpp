/**
 * @file localPlanner.cpp
 * @author 李东权 (1327165187@qq.com)
 * @brief
 * @version 0.1
 * @date 2024-02-23
 *
 * @copyright Copyright (c) 2024
 *
 */
#include "LpNode.h"
#include <chrono>
int need_pub_Map = 100;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "localPlanner");
    LpNode LocalPanner;          // 实例化局部规划器
    LocalPanner.read_pathfile(); // 初始化容器，并读取保存各个path数据文件
    ros::Rate rate(10);
    bool status = ros::ok();

    while (status)
    {
        auto start = std::chrono::high_resolution_clock::now();
        /**
         * @brief 外部参数更新，选择地图后再完成回调
         *
         */
        LocalPanner.lctlPtr->update_params();
        ros::spinOnce();
        /**
         * @brief 相关参数更新，需要读取路径文件
         *
         */
        if (LocalPanner.need_read_path)
        {
            LocalPanner.read_pathfile();
            LocalPanner.need_read_path = false;
        }
        // 可视化pcd地图
        need_pub_Map--;
        if (need_pub_Map == 0)
        {
            LocalPanner.pub_Map();
            need_pub_Map = 100;
        }
        /**
         * @brief 外部请求关闭地图相关机制
         *
         */
        // 关闭地图已到时，重新打开地图
        if (!LocalPanner.lctlPtr->get_params().use_map && !(abs(LocalPanner.close_map_begin - (-1)) < 0.01))
        {
            double close_map_end = ros::Time::now().toSec();
            double close_map_duration = close_map_end - LocalPanner.close_map_begin;
            if (close_map_duration > LocalPanner.lctlPtr->get_params().close_map_time)
            {
                LocalPanner.lctlPtr->set_use_map(true);
            }
        }
        else // 外部请求关闭
        {
            if (LocalPanner.need_close_map)
            {
                LocalPanner.close_map();
                LocalPanner.need_close_map = false;
            }
        }
        /**
         * @brief 有新的地形点云，进行路径规划
         *
         */
        if (newTerrainCloud)
        {
            pathFound = false;
            newTerrainCloud = false;
            LocalPanner.transform_allCloud();   // 转换所有点云到车头x方向的车体坐标系，并按照距离过滤
            LocalPanner.pathRange_from_speed(); // 根据速度动态调整路径范围
            LocalPanner.transform_goal();       // 在自动模式下, 将目标点转移到车体坐标系下,并记录下目标相对车体的yaw角
            LocalPanner.local_planner();        // 正式进行路径规划
            if (!pathFound)
            {
                if (LocalPanner.lctlPtr->get_params().use_fail_closemap)
                    LocalPanner.close_map();
                LocalPanner.fail_local_planner(); // 找不到路径，发布一个原地的路径点
            }
        }
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
        // 静态变量降低打印频率
        static int print_count = 0;
        if (print_count % 10 == 0)
        {
            ROS_INFO("run one localPlanner time: %f ms", duration.count() / 1000.0);
            print_count = 0;
        }
        print_count++;
        
        status = ros::ok();
        rate.sleep();
    }

    return 0;
}
