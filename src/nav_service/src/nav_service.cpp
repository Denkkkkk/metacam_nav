#include <string>
#include <vector>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Trigger.h>

#include "types.h"
#include <json.hpp>
#include <queue>

bool is_running = false;
NavigationModel nav_model;
double vehicle_x = 0.0;
double vehicle_y = 0.0;
int nav_index = 0;
/**
 * @brief 更新导航点参数
 *
 * @param req
 * @param res
 * @return true
 * @return false
 */
bool updateConfigCallback(std_srvs::Trigger::Request &req,
                          std_srvs::Trigger::Response &res)
{
    std::string config;
    if (ros::param::get("/nav/config", config))
    // if (true)
    {
        nlohmann::json json_data = nlohmann::json::parse(config);
        nav_model = json_data.get<NavigationModel>();
        res.success = true;
        res.message = "get navigation config successfully!";

        nav_index = 0;
    }
    else
    {
        res.success = false;
        res.message = "fail to get navigation config since /nav/config is not given!";
    }
    ROS_WARN("config:%s", config.c_str());

    if (res.success)
        ROS_INFO("%s", res.message);
    else
        ROS_ERROR("%s", res.message);
    return true;
}

/**
 * @brief 开启导航服务
 *
 * @param req
 * @param res
 * @return true
 * @return false
 */
bool startCallback(std_srvs::Trigger::Request &req,
                   std_srvs::Trigger::Response &res)
{
    if (is_running)
    {
        res.success = false;
        res.message = "fail to request start navigation since it has been started!";
    }
    else if (updateConfigCallback(req, res))
    {
        res.success = true;
        res.message = "request start navigation successfully!";
        is_running = true;
    }

    if (res.success)
        ROS_INFO("%s", res.message);
    else
        ROS_ERROR("%s", res.message);
    return true;
}

/**
 * @brief 关闭导航服务
 *
 * @param req
 * @param res
 * @return true
 * @return false
 */
bool stopCallback(std_srvs::Trigger::Request &req,
                  std_srvs::Trigger::Response &res)
{
    if (!is_running)
    {
        res.success = false;
        res.message = "fail to request stop record since it has been stopped!";
    }
    else
    {
        res.success = true;
        res.message = "request stop navigation successfully!";
        is_running = false;
    }

    if (res.success)
        ROS_INFO("%s", res.message);
    else
        ROS_ERROR("%s", res.message);
    return true;
}

void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    vehicle_x = msg->pose.pose.position.x;
    vehicle_y = msg->pose.pose.position.y;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "nav_service");
    ros::NodeHandle nh;
    ros::ServiceServer update_config_service = nh.advertiseService("/nav/update_config", updateConfigCallback);
    ros::ServiceServer start_service = nh.advertiseService("/nav/start", startCallback);
    ros::ServiceServer stop_service = nh.advertiseService("/nav/stop", stopCallback);
    // 里程计订阅
    ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>("/Odometry", 2, odomCallback);
    // 发布way_point
    ros::Publisher way_point_pub = nh.advertise<geometry_msgs::PoseStamped>("/way_point", 2);
    // 发布停止
    ros::Publisher stop_pub = nh.advertise<std_msgs::Bool>("/stop", 2);

    ros::Rate loop_rate(20);
    geometry_msgs::PoseStamped way_point;
    while (ros::ok())
    {
        loop_rate.sleep();
        ros::spinOnce();
        // 导航点出栈并发布到way_point，到点后再出栈下一个
        if (is_running)
        {
            // 恢复停止标志位
            std_msgs::Bool stop;
            stop.data = false;
            stop_pub.publish(stop);
            if (nav_model.points.empty())
            {
                ROS_INFO("NO Navigation Point!");
                continue;
            }
            if (!nav_model.parameters.empty() && nav_model.parameters[0] > 0)
            {
                const auto &point = nav_model.points.at(nav_index);
                // way_point赋值
                way_point.header.frame_id = "map";
                way_point.header.stamp = ros::Time::now();
                way_point.pose.position.x = point.x;
                way_point.pose.position.y = point.y;
                way_point.pose.position.z = point.z;
                way_point.pose.orientation.x = 0;
                way_point.pose.orientation.y = 0;
                way_point.pose.orientation.z = 0;
                way_point.pose.orientation.w = 1;
                way_point_pub.publish(way_point);
            }
            // 判断到点状态
            double dis = sqrt(pow(vehicle_x - way_point.pose.position.x, 2) + pow(vehicle_y - way_point.pose.position.y, 2));
            if (dis < 0.3)
            {
                nav_index += 1;
                if (nav_index == nav_model.points.size())
                {
                    nav_model.parameters[0] -= 1;
                    nav_index = 0;
                }
            }
            else
            {
                way_point_pub.publish(way_point); // 未到点不断发布
                ROS_INFO("points_num: %ld, left_count: %.0f", nav_model.points.size(), nav_model.parameters[0]);
                ROS_INFO("Going to Navigation Point: x: %f, y: %f, z: %f", way_point.pose.position.x, way_point.pose.position.y, way_point.pose.position.z);
            }
        }
        else
        {
            // 发布停止标志位
            std_msgs::Bool stop;
            stop.data = true;
            stop_pub.publish(stop);
            ROS_INFO("Navigation service is not running!");
        }
    }
    return 0;
}
