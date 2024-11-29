#include "nav_service/parameters.h"
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <string>
#include <tf/tf.h>
#include <vector>

#include "types.h"
#include <json.hpp>
#include <queue>

bool is_running = false;
NavigationModel nav_model;
double vehicle_x = 0.0;
double vehicle_y = 0.0;
int nav_index = 0;
std::vector<Coordinate> nav_path;
std::vector<Coordinate> nav_mode_path;

std::vector<Coordinate> get_path_from_nav_model(const NavigationModel &nav_model)
{
    std::vector<Coordinate> path;
    for (const auto &point : nav_model.points)
    {
        Coordinate coord;
        coord.x = point.x;
        coord.y = point.y;
        coord.yaw = 0;
        path.push_back(coord);
    }
    return path;
}

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
    {
        ROS_INFO("%s", res.message.c_str());
    }
    else
    {
        ROS_ERROR("%s", res.message.c_str());
    }
    return res.success;
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
    // 输出提示信息
    ROS_INFO("start navigation service!");

    if (is_running)
    {
        res.success = false;
        res.message = "fail to request start navigation since it has been started!";
    }
    else
    {
        updateConfigCallback(req, res);
        res.success = true;
        res.message = "request start navigation successfully!";
        is_running = true;
    }

    if (res.success)
        ROS_INFO("%s", res.message.c_str());
    else
        ROS_ERROR("%s", res.message.c_str());
    return res.success;
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
        ROS_INFO("%s", res.message.c_str());
    else
        ROS_ERROR("%s", res.message.c_str());
    return true;
}

void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    vehicle_x = msg->pose.pose.position.x;
    vehicle_y = msg->pose.pose.position.y;
}

void navStatusPub(ros::Publisher &nav_status_pub)
{
    NavStatus nav_status;
    nav_status.version = "v1.3.5";
    nav_status.is_running = is_running;
    nav_status.target_index = nav_index;
    if (!nav_path.empty())
    {
        const auto &point = nav_path.at(nav_index);
        nav_status.target_pose = {point.x, point.y, point.z, 0, 0, 0};
    }
    else
    {
        nav_status.target_pose = {0, 0, 0, 0, 0, 0};
    }
    std::string json_str;
    NavStatus_to_json(nav_status, json_str);
    std_msgs::String status;
    status.data = json_str;
    nav_status_pub.publish(status);
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
    // 发布导航状态
    ros::Publisher nav_status_pub = nh.advertise<std_msgs::String>("/nav/status", 2);

    ros::Rate loop_rate(10);
    ParamControl nav_service_params;
    geometry_msgs::PoseStamped way_point;
    // 初始化nav_model
    nav_model.mode = 0;
    nav_model.points = {};
    nav_model.parameters = {0};

    while (ros::ok())
    {
        loop_rate.sleep();
        ros::spinOnce();
        nav_service_params.update_params();
        if (nav_service_params.get_params().use_prior_path)
        {
            nav_path = nav_service_params.get_params().prior_path;
        }
        else
        {
            nav_path = get_path_from_nav_model(nav_model);
        }
        navStatusPub(nav_status_pub); // 发布导航状态

        // 导航点取出并发布到way_point，到点后再指向下一个
        if (is_running)
        {
            if (nav_path.empty())
            {
                ROS_ERROR("NO Navigation Point!");
                // 强制停止
                std_msgs::Bool stop;
                stop.data = true;
                stop_pub.publish(stop);
                continue;
            }
            else if (nav_model.parameters.empty() && !nav_service_params.get_params().use_prior_path)
            {
                ROS_ERROR("nav_model Parameters Is Empty!");
                // 强制停止
                std_msgs::Bool stop;
                stop.data = true;
                stop_pub.publish(stop);
                continue;
            }
            else if (nav_model.parameters[0] <= 0 && !nav_service_params.get_params().use_prior_path)
            {
                ROS_ERROR("nav_model.parameters[0] :%f !", nav_model.parameters[0]);
                // 强制停止
                std_msgs::Bool stop;
                stop.data = true;
                stop_pub.publish(stop);
                continue;
            }
            else
            {
                // 恢复停止标志位
                std_msgs::Bool stop;
                stop.data = false;
                stop_pub.publish(stop);
                const auto &point = nav_path.at(nav_index);
                // way_point赋值
                way_point.header.frame_id = "map";
                way_point.header.stamp = ros::Time::now();
                way_point.pose.position.x = point.x;
                way_point.pose.position.y = point.y;
                if (nav_service_params.get_params().use_prior_path)
                {
                    way_point.pose.position.z = 0;
                    // 一行代码转换yaw角为四元数
                    way_point.pose.orientation = tf::createQuaternionMsgFromYaw(point.yaw);
                }
                else
                {
                    way_point.pose.position.z = -1;
                    way_point.pose.orientation.x = 0;
                    way_point.pose.orientation.y = 0;
                    way_point.pose.orientation.z = 0;
                    way_point.pose.orientation.w = 1;
                }
                way_point_pub.publish(way_point);
            }
            // 判断到点状态
            double dis = sqrt(pow(vehicle_x - way_point.pose.position.x, 2) + pow(vehicle_y - way_point.pose.position.y, 2));
            if (dis < nav_service_params.get_params().endGoalDis)
            {
                nav_index += 1;
                if (nav_index == nav_path.size())
                {
                    nav_model.parameters[0] -= 1;
                    nav_index = 0;
                }
                ROS_INFO("get point: x: %f, y: %f, z: %f", way_point.pose.position.x, way_point.pose.position.y, way_point.pose.position.z);
            }
            else
            {
                way_point_pub.publish(way_point); // 未到点不断发布
                ROS_INFO("points_num: %ld, left_count: %.0f", nav_path.size(), nav_model.parameters[0]);
                ROS_INFO("Going to Navigation Point: x: %f, y: %f, z: %f", way_point.pose.position.x, way_point.pose.position.y, way_point.pose.position.z);
            }
        }
        else
        {
            // 发布停止标志位
            std_msgs::Bool stop;
            stop.data = true;
            stop_pub.publish(stop);
            ROS_ERROR("Navigation service is not running!");
        }
    }
    return 0;
}
