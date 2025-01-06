#ifndef NAV_LOGGING_H
#ifndef SPD_LOGGING_H
#define SPD_LOGGING_H
#include "spdlog/spdlog.h"
#include "ros/ros.h"
template <typename... Args>
using format_string_t = fmt::format_string<Args...>;

void logWithColor(const char* color, const char* node_name, const char* timeStr, const char* msg);

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
    const char* node_name = ros::this_node::getName().c_str();
    if (!node_name)
    {
        node_name = "Unknown_Node";
    }
    spdlog::info(fmt, std::forward<Args>(args)...);
}
void NAV_WARNING(const char* msg);
void NAV_PASS(const char* msg);
void NAV_ERROR(const char* msg);
void NAV_INFO(const char* msg);

#endif // SPD_LOGGING_H
#else
#error "nav_logging.h is already included, please do not include both spd_logging.h and nav_logging.h"
#endif // NAV_LOGGING_H