#include "types.h"
#include <fstream>
#include <geometry_msgs/PoseStamped.h>
#include <json.hpp>
#include <nav_msgs/Odometry.h>
#include <queue>
#include <ros/package.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <string>
#include <vector>

NavigationModel nav_model;
double vehicle_x = 0.0;
double vehicle_y = 0.0;
ros::ServiceClient client;
std::string local_config;
std::string cached_json_str = ""; // 缓存的 JSON 字符串

/**
 * @brief 开启导航服务
 *
 * @param req
 * @param res
 * @return true
 * @return false
 */
void update_param()
{
    // 1. 打开 JSON 文件
    std::ifstream input_file(local_config);
    if (!input_file.is_open())
    {
        std::cerr << "Could not open the file!" << std::endl;
        return;
    }

    // 解析 JSON 数据
    nlohmann::json j;
    input_file >> j; // 读取文件内容到 JSON 对象 j

    // 将 JSON 对象转换为字符串
    std::string json_str = j.dump();

    // 2. 比较当前读取的 JSON 字符串与缓存的内容
    if (json_str != cached_json_str) 
    {
        // 3. 如果文件内容发生变化，更新 ROS 参数
        ros::param::set("/nav/config", json_str);

        // 更新缓存
        cached_json_str = json_str;

        // 创建请求和响应对象
        std_srvs::Trigger srv;

        // 调用服务
        if (client.call(srv))
        {
            // ROS_INFO("Service call successful. Response: %s", srv.response.message.c_str());
        }
        else
        {
            ROS_ERROR("Failed to call service /nav/update_config");
        }
    }
    else
    {
        // 文件没有发生变化，跳过更新
        ROS_INFO("No change in JSON data. Skipping update.");
    }
    return;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "debug_node");
    ros::NodeHandle nh;
    client = nh.serviceClient<std_srvs::Trigger>("/nav/update_config");

    local_config = ros::package::getPath("debug_node") + "/config/nav_mode.json";

    ros::Rate loop_rate(1);
    // 初始化nav_model
    nav_model.mode = 0;
    nav_model.points = {};
    nav_model.parameters = {0};
    while (ros::ok())
    {
        ros::spinOnce();
        update_param();
        loop_rate.sleep();
    }
    return 0;
}
