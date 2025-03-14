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
#include <Eigen/Dense>
#include <json.hpp>
#include <queue>
#include "navlog_control/spd_logging.hpp"

bool is_running = false;
NavigationModel nav_model;
double vehicle_x = 0.0;
double vehicle_y = 0.0;
int *nav_index;
int queue_path_index = 0;
int prior_path_index = 0;
std::vector<Coordinate> nav_path;
std::vector<Coordinate> nav_mode_path;
double get_relocal_begin = -100;
bool reloc_succeed = false;
Eigen::Isometry3d odom_to_map;

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
    // 从 ROS 参数服务器获取名为 /nav/config 的参数值，并将其存储在 config 变量中
    if (ros::param::get("/nav/config", config))
    {
        nlohmann::json json_data = nlohmann::json::parse(config);
        // 装进nav_model
        nav_model = json_data.get<NavigationModel>();
        res.success = true;
        res.message = "get navigation config successfully!";
        // 将全部点都乘odom_to_map，装回到nav_model
        // 里程计坐标系转换到地图坐标系
        if (nav_model.points.size() > 0 && nav_model.points[0].x != 99999)
        {
            for (auto &point : nav_model.points)
            {
                Eigen::Vector3d point_vector(point.x, point.y, point.z);
                point_vector = odom_to_map * point_vector;
                point.x = point_vector.x();
                point.y = point_vector.y();
                point.z = point_vector.z();
            }
        }

        if (queue_path_index >= nav_model.points.size())
        {
            queue_path_index = nav_model.points.size() - 1;
        }
        NAV_WARN_ONCE("nav_service config update:{}", config.c_str());
    }
    else
    {
        res.success = false;
        res.message = "fail to get navigation config since /nav/config is not given!";
    }


    if (res.success)
    {
        DEBUG_NAV_INFO("{}", res.message.c_str());
    }
    else
    {
        NAV_ERROR_ONCE("{}", res.message.c_str());
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
        DEBUG_NAV_INFO("{}", res.message.c_str());
    else
        NAV_ERROR_ONCE("{}", res.message.c_str());
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
    reloc_succeed = false;
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
        DEBUG_NAV_INFO("{}", res.message.c_str());
    else
        NAV_ERROR_ONCE("{}", res.message.c_str());
    return true;
}

/**
 * @brief 里程计回调函数，获取里程计数据
 *
 * @param msg
 */
void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    vehicle_x = msg->pose.pose.position.x;
    vehicle_y = msg->pose.pose.position.y;
}

/**
 * @brief 发布导航状态
 *
 * @param nav_status_pub
 */
void navStatusPub(ros::Publisher &nav_status_pub)
{
    NavStatus nav_status;
    nav_status.version = "v1.4.8";
    nav_status.is_running = is_running;
    nav_status.target_index = *nav_index;
    if (!nav_path.empty())
    {
        const auto &point = nav_path.at(nav_status.target_index);
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

void reLocalizationCallBack(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    get_relocal_begin = ros::Time::now().toSec();
    NAV_INFO_ONCE("reLocalizationCallBack");
    reloc_succeed = true;
}

/**
 * @brief 里程计转地图坐标系
 *
 * @param msg
 */
void odomToMapCallBack(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    odom_to_map.setIdentity(); // 重置为单位矩阵，否则会发生累积变换
    // 将里程计转成Eigen::Isometry3d
    double roll, pitch, yaw;
    geometry_msgs::Quaternion geoQuat = msg->pose.orientation;
    tf::Matrix3x3(tf::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w)).getEulerYPR(yaw, pitch, roll);
    Eigen::Matrix3d odom_to_map_rotation;
    odom_to_map_rotation = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
                           Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                           Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
    odom_to_map.rotate(odom_to_map_rotation);
    odom_to_map.pretranslate(Eigen::Vector3d(msg->pose.position.x,
                                             msg->pose.position.y,
                                             msg->pose.position.z));
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "nav_service");
    INIT_NAVLOG_ROS();
    ros::NodeHandle nh;
    ros::ServiceServer update_config_service = nh.advertiseService("/nav/update_config", updateConfigCallback);
    ros::ServiceServer start_service = nh.advertiseService("/nav/start", startCallback);
    ros::ServiceServer stop_service = nh.advertiseService("/nav/stop", stopCallback);
    // 里程计订阅
    ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>("/odom_interface", 2, odomCallback);
    ros::Subscriber subReLocal = nh.subscribe<geometry_msgs::PoseStamped>("/relocalization", 5, reLocalizationCallBack);
    ros::Subscriber subOdomToMap = nh.subscribe<geometry_msgs::PoseStamped>("/odomToMapPose", 5, odomToMapCallBack);
    // 发布goal_point
    ros::Publisher goal_point_pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 5);
    // 发布停止
    ros::Publisher stop_pub = nh.advertise<std_msgs::Bool>("/stop", 2);
    // 发布导航状态
    ros::Publisher nav_status_pub = nh.advertise<std_msgs::String>("/nav/status", 2);
    ros::Publisher nav_relo_pub = nh.advertise<std_msgs::Bool>("/need_reloc", 1);

    ros::Rate loop_rate(50);
    NAV_WARN_ONCE("nav_service node start init!");
    ParamControl nav_service_params;
    geometry_msgs::PoseStamped goal_point;
    // 初始化nav_model
    nav_model.mode = 0;
    nav_model.points = {};
    nav_model.parameters = {0};
    // 初始化odom_to_map为单位矩阵
    odom_to_map.setIdentity();

    while (ros::ok())
    {
        loop_rate.sleep();
        ros::spinOnce();
        nav_service_params.update_params();
        if (nav_service_params.get_params().use_prior_path && (!nav_model.points.empty() && abs(nav_model.points[0].x - 99999) < 0.1))
        {
            nav_path = nav_service_params.get_params().prior_path;
            nav_index = &prior_path_index;
        }
        else
        {
            nav_path = get_path_from_nav_model(nav_model);
            nav_index = &queue_path_index;
        }
        navStatusPub(nav_status_pub); // 发布导航状态

        // 导航点取出并发布到way_point，到点后再指向下一个
        if ((nav_model.mode !=1) && (is_running || nav_model.mode == 2) )
        {
            if (nav_service_params.get_params().use_relocalization && (!nav_model.points.empty() && abs(nav_model.points[0].x - 99999) < 0.1))
            {
                double get_relocal_duration = ros::Time::now().toSec() - get_relocal_begin;
                if (!reloc_succeed)
                {
                    std_msgs::Bool relo;
                    relo.data = true;
                    nav_relo_pub.publish(relo);
                    NAV_ERROR("Navigation need relocalization!");
                    // sleep 1.0s
                    ros::Duration(1.0).sleep();
                    continue;
                }
                else
                {
                    if (get_relocal_duration < 1.5 && get_relocal_duration > 0)
                    {
                        continue;
                    }
                }
            }
            if (nav_path.empty())
            {
                NAV_ERROR("NO Navigation Point!");
                // 强制停止
                std_msgs::Bool stop;
                stop.data = true;
                stop_pub.publish(stop);
                continue;
            }
            else if (nav_model.parameters.empty() && !(nav_service_params.get_params().use_prior_path && abs(nav_model.points[0].x - 99999) < 0.1))
            {
                NAV_ERROR("nav_model Parameters Is Empty!");
                // 强制停止
                std_msgs::Bool stop;
                stop.data = true;
                stop_pub.publish(stop);
                continue;
            }
            else if (nav_model.parameters[0] <= 0 && !(nav_service_params.get_params().use_prior_path && abs(nav_model.points[0].x - 99999) < 0.1))
            {
                NAV_ERROR("NO nav_times, nav_model.parameters[0] :{} !", nav_model.parameters[0]);
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
                const auto &point = nav_path.at(*nav_index);
                // way_point赋值
                goal_point.header.frame_id = "map";
                goal_point.header.stamp = ros::Time::now();
                goal_point.pose.position.x = point.x;
                goal_point.pose.position.y = point.y;
                if (nav_service_params.get_params().use_prior_path && (!nav_model.points.empty() && abs(nav_model.points[0].x - 99999) < 0.1))
                {
                    goal_point.pose.position.z = 0;
                    // 一行代码转换yaw角为四元数
                    goal_point.pose.orientation = tf::createQuaternionMsgFromYaw(point.yaw);
                }
                else
                {
                    goal_point.pose.position.z = -1;
                    goal_point.pose.orientation.x = 0;
                    goal_point.pose.orientation.y = 0;
                    goal_point.pose.orientation.z = 0;
                    goal_point.pose.orientation.w = 1;
                }
                goal_point_pub.publish(goal_point);
            }
            // 判断到点状态
            double dis = sqrt(pow(vehicle_x - goal_point.pose.position.x, 2) + pow(vehicle_y - goal_point.pose.position.y, 2));
            if (dis < nav_service_params.get_params().endGoalDis)
            {
                ros::Duration(nav_service_params.get_params().endGoal_stopTime).sleep();
                *nav_index += 1;
                if (*nav_index == nav_path.size())
                {
                    if (nav_model.parameters[0] >= 0)
                    {
                        nav_model.parameters[0] -= 1;
                    }
                    *nav_index = 0;
                }
                NAV_INFO("get point: x: {}, y: {}, z: {}", goal_point.pose.position.x, goal_point.pose.position.y, goal_point.pose.position.z);
            }
            else
            {
                goal_point_pub.publish(goal_point); // 未到点不断发布
                NAV_INFO("points_num: {}, left_count: {}", nav_path.size(), nav_model.parameters[0]);
                NAV_INFO("Going to Navigation Point: x: {}, y: {], z: {}", goal_point.pose.position.x, goal_point.pose.position.y, goal_point.pose.position.z);
            }
        }
        else
        {
            // 发布停止标志位
            std_msgs::Bool stop;
            stop.data = true;
            stop_pub.publish(stop);
            NAV_ERROR("Navigation service is not running!");
        }
    }
    return 0;
}
