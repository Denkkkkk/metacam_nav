#include "msg_process/receive_data.h"
#include "start_control/start_control.h"
using namespace std;

// 2D雷达重启
int main(int argc, char **argv)
{
    ros::init(argc, argv, "start_control");
    StartControl sc;
    ros::Rate rate(100);
    bool status = ros::ok();
    int times = 0;
    bool get_points = false;
    while (status)
    {
        // 处理所有的订阅回调，先拉取各种的数据信息，点云信息
        ros::spinOnce();
        bool status = ros::ok();
        if (!sc.rplidar_status.data)
        {
            ROS_WARN("noRplidar_client.....");
            rate.sleep();
            continue;
        }
        if (sc.lidar_points->size() > 5)
        {
            get_points = true;
        }
        if (get_points)
        {
            // ROS_WARN("get_rplidar!!!");
            rate.sleep();
            continue;
        }
        if (times > 400)
        {
            int status = system("gnome-terminal -- bash -c 'rosnode kill /rplidarNode;roslaunch rplidar_ros rplidar.launch'");
            ROS_ERROR("reStarting_rplidar.....");
            ros::Duration(2).sleep();
            times = 0;
        }
        else
        {
            times++;
        }
        ROS_WARN("waiting_rplidar_points.....");
        rate.sleep();
    }
    return 0;
}