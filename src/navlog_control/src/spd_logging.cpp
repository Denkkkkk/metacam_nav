#include "navlog_control/spd_logging.h"

void logWithColor(const char* color, const char* node_name, const char* timeStr, const char* msg)
{
    // 打印带有颜色的日志
    printf("%s[%s] [%s] %s\033[0m\n", color, node_name, timeStr, msg);
}


void NAV_HIGHLIGHT(const char* msg)
{
    // 获取当前时间
    time_t rawtime;
    struct tm *timeinfo;
    char timeStr[20];
    time(&rawtime);
    timeinfo = localtime(&rawtime);
    strftime(timeStr, sizeof(timeStr), "%m-%d %H:%M:%S", timeinfo);

    // 获取当前 ROS 节点名称
    const char* node_name = ros::this_node::getName().c_str();
    if (!node_name)
    {
        node_name = "Unknown_Node";
    }
    spdlog::info(node_name,msg);
}

void NAV_WARNING(const char* msg)
{
    // 获取当前时间
    time_t rawtime;
    struct tm *timeinfo;
    char timeStr[20];
    time(&rawtime);
    timeinfo = localtime(&rawtime);
    strftime(timeStr, sizeof(timeStr), "%m-%d %H:%M:%S", timeinfo);

    // 获取当前 ROS 节点名称
    const char* node_name = ros::this_node::getName().c_str();
    if (!node_name)
    {
        node_name = "Unknown_Node";
    }

    // 使用黄色输出日志
    logWithColor("\033[33m", node_name, timeStr, msg);
}

void NAV_PASS(const char* msg)
{
    // 获取当前时间
    time_t rawtime;
    struct tm *timeinfo;
    char timeStr[20];
    time(&rawtime);
    timeinfo = localtime(&rawtime);
    strftime(timeStr, sizeof(timeStr), "%m-%d %H:%M:%S", timeinfo);

    // 获取当前 ROS 节点名称
    const char* node_name = ros::this_node::getName().c_str();
    if (!node_name)
    {
        node_name = "Unknown_Node";
    }

    // 使用绿色输出日志
    logWithColor("\033[32m", node_name, timeStr, msg);
}

void NAV_ERROR(const char* msg)
{
    // 获取当前时间
    time_t rawtime;
    struct tm *timeinfo;
    char timeStr[20];
    time(&rawtime);
    timeinfo = localtime(&rawtime);
    strftime(timeStr, sizeof(timeStr), "%m-%d %H:%M:%S", timeinfo);

    // 获取当前 ROS 节点名称
    const char* node_name = ros::this_node::getName().c_str();
    if (!node_name)
    {
        node_name = "Unknown_Node";
    }

    // 使用红色输出日志
    logWithColor("\033[31m", node_name, timeStr, msg);
}

void NAV_INFO(const char* msg)
{
    // 获取当前时间
    time_t rawtime;
    struct tm *timeinfo;
    char timeStr[20];
    time(&rawtime);
    timeinfo = localtime(&rawtime);
    strftime(timeStr, sizeof(timeStr), "%m-%d %H:%M:%S", timeinfo);

    // 获取当前 ROS 节点名称
    const char* node_name = ros::this_node::getName().c_str();
    if (!node_name)
    {
        node_name = "Unknown_Node";
    }

    // 输出普通日志
    logWithColor("", node_name, timeStr, msg);
}