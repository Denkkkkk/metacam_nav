#ifndef NAV_LOGGING_H
#ifndef SPD_LOGGING_H
#define SPD_LOGGING_H
#include "navlog_control/spd_class.h"
#include "ros/ros.h"
#include "spdlog/spdlog.h"
#include <functional>
#include <unordered_map>

template <typename... Args>
using format_string_t = fmt::format_string<Args...>;

#define CREATE_STATIC()   \
    do                    \
    {                     \
        static int i = 1; \
        if (i++ < 10)     \
        {                 \
            return;       \
        }                 \
        i = 1;            \
    } while (false)

template <typename... Args>
void NAV_HIGHLIGHT(format_string_t<Args...> fmt, Args &&...args)
{
    // 获取当前时间
    time_t rawtime;
    struct tm *timeinfo;
    char timeStr[20];
    time(&rawtime);
    timeinfo = localtime(&rawtime);
    strftime(timeStr, sizeof(timeStr), "%m-%d %H:%M:%S", timeinfo);

    // 获取当前 ROS 节点名称
    const char *node_name = ros::this_node::getName().c_str();
    if (!node_name)
    {
        node_name = "Unknown_Node";
    }

    CREATE_STATIC();
    spdClass.file_logger->critical(fmt, std::forward<Args>(args)...);
    spdClass.my_logger->critical(fmt, std::forward<Args>(args)...);
//    spdlog::critical(fmt, std::forward<Args>(args)...);
}

template <typename... Args>
void NAV_WARNING(format_string_t<Args...> fmt, Args &&...args)
{
    // 获取当前时间
    time_t rawtime;
    struct tm *timeinfo;
    char timeStr[20];
    time(&rawtime);
    timeinfo = localtime(&rawtime);
    strftime(timeStr, sizeof(timeStr), "%m-%d %H:%M:%S", timeinfo);

    // 获取当前 ROS 节点名称
    const char *node_name = ros::this_node::getName().c_str();
    if (!node_name)
    {
        node_name = "Unknown_Node";
    }

    CREATE_STATIC();
    spdClass.file_logger->warn(fmt, std::forward<Args>(args)...);
    spdlog::warn(fmt, std::forward<Args>(args)...);
}

template <typename... Args>
void NAV_DEBUG(format_string_t<Args...> fmt, Args &&...args)
{
    // 获取当前时间
    time_t rawtime;
    struct tm *timeinfo;
    char timeStr[20];
    time(&rawtime);
    timeinfo = localtime(&rawtime);
    strftime(timeStr, sizeof(timeStr), "%m-%d %H:%M:%S", timeinfo);

    // 获取当前 ROS 节点名称
    const char *node_name = ros::this_node::getName().c_str();
    if (!node_name)
    {
        node_name = "Unknown_Node";
    }

    CREATE_STATIC();
    spdClass.file_logger->debug(fmt, std::forward<Args>(args)...);
    spdlog::debug(fmt, std::forward<Args>(args)...);
}

template <typename... Args>
void NAV_ERROR(format_string_t<Args...> fmt, Args &&...args)
{
    // 获取当前时间
    time_t rawtime;
    struct tm *timeinfo;
    char timeStr[20];
    time(&rawtime);
    timeinfo = localtime(&rawtime);
    strftime(timeStr, sizeof(timeStr), "%m-%d %H:%M:%S", timeinfo);

    // 获取当前 ROS 节点名称
    const char *node_name = ros::this_node::getName().c_str();
    if (!node_name)
    {
        node_name = "Unknown_Node";
    }

    CREATE_STATIC();
    spdClass.file_logger->error(fmt, std::forward<Args>(args)...);
    spdlog::error(fmt, std::forward<Args>(args)...);
}

template <typename... Args>
void NAV_INFO(format_string_t<Args...> fmt, Args &&...args)
{
    // 获取当前时间
    time_t rawtime;
    struct tm *timeinfo;
    char timeStr[20];
    time(&rawtime);
    timeinfo = localtime(&rawtime);
    strftime(timeStr, sizeof(timeStr), "%m-%d %H:%M:%S", timeinfo);

    // 获取当前 ROS 节点名称
    const char *node_name = ros::this_node::getName().c_str();
    if (!node_name)
    {
        node_name = "Unknown_Node";
    }

    CREATE_STATIC();
    spdClass.file_logger->info(fmt, std::forward<Args>(args)...);
    spdlog::info(fmt, std::forward<Args>(args)...);
}

#endif // SPD_LOGGING_H
#else
#error "nav_logging.h is already included, please do not include both spd_logging.h and nav_logging.h"
#endif // NAV_LOGGING_H