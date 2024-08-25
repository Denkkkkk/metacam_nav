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
std::queue<Point> need_nav_points;
double vehicle_x = 0.0;
double vehicle_y = 0.0;
bool need_pop_navPoint = false;
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
    // if (ros::param::get("/nav/update_config", config))
    if (true)
    {
        // nlohmann::json json_data = nlohmann::json::parse(config);
        // nav_model = json_data.get<NavigationModel>();
        // res.success = true;
        // res.message = "get navigation config successfully!";
        /**
         * @brief 导航点入栈处理
         *
         */
        // 清空队列内数据
        while (!need_nav_points.empty())
        {
            need_nav_points.pop();
        }
        // 按序入队
        // 点序列调试模式
        res.success = true;
        Point point1 = {1, 0, 0};
        Point point2 = {2, 2, 0};
        need_nav_points.push(point1);
        need_nav_points.push(point2);
        // for (auto &point : nav_model.points)
        // {

        //     need_nav_points.push(point);
        // }
        need_pop_navPoint = true;
    }
    else
    {
        res.success = false;
        res.message = "fail to get navigation config since /nav/update_config is not given!";
    }

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

    ros::Rate loop_rate(50);
    geometry_msgs::PoseStamped way_point;
    while (ros::ok())
    {
        ros::spinOnce();
        // 导航点出栈并发布到way_point，到点后再出栈下一个
        if (is_running)
        {
            // 恢复停止标志位
            std_msgs::Bool stop;
            stop.data = false;
            stop_pub.publish(stop);
            if (need_nav_points.empty())
            {
                ROS_INFO("All Navigation Point is finished!");
                continue;
            }
            // 是否需要取出队首元素并发布
            if (!need_nav_points.empty() && need_pop_navPoint)
            {
                Point point = need_nav_points.front(); // 获取队首元素
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
                need_pop_navPoint = false;
            }
            // 判断到点状态
            double dis = sqrt(pow(vehicle_x - way_point.pose.position.x, 2) + pow(vehicle_y - way_point.pose.position.y, 2));
            if (dis < 0.3)
            {
                need_nav_points.pop(); // 到点后出队
                need_pop_navPoint = true;
            }
            else
            {
                way_point_pub.publish(way_point); // 未到点不断发布
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

        loop_rate.sleep();
    }
    return 0;
}
