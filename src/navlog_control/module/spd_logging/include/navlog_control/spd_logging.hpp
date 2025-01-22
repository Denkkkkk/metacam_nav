/**
 * @file spd_logging.hpp
 * @author 李东权 1327165187@qq.com
 * @brief 重新实现spdlog库的日志打印函数
 * @version 1.0
 * @date 2025-01-22
 * 
 * @copyright Copyright (c) 2025
 * 
 */
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
inline void NAV_HIGHLIGHT(format_string_t<Args...> fmt, Args &&...args)
{
    CREATE_STATIC();
    spdClass.file_logger->set_level(spdlog::level::critical);
    spdClass.my_logger->set_level(spdlog::level::critical);
    spdClass.file_logger->critical(fmt, std::forward<Args>(args)...);
    spdClass.my_logger->critical(fmt, std::forward<Args>(args)...);
}

template <typename... Args>
inline void NAV_WARNING(format_string_t<Args...> fmt, Args &&...args)
{
    CREATE_STATIC();
    spdClass.file_logger->set_level(spdlog::level::warn);
    spdClass.my_logger->set_level(spdlog::level::warn);
    spdClass.file_logger->warn(fmt, std::forward<Args>(args)...);
    spdClass.my_logger->warn(fmt, std::forward<Args>(args)...);
}

template <typename... Args>
inline void NAV_INFO(format_string_t<Args...> fmt, Args &&...args)
{
    // 获取当前 ROS 节点名称
    const char *node_name = ros::this_node::getName().c_str();
    if (!node_name)
    {
        node_name = "Unknown_Node";
    }

    CREATE_STATIC();
    spdClass.file_logger->set_level(spdlog::level::trace);
    spdClass.my_logger->set_level(spdlog::level::trace);
    spdClass.file_logger->trace(fmt, std::forward<Args>(args)...);
    spdClass.my_logger->trace(fmt, std::forward<Args>(args)...);
}

template <typename... Args>
inline void NAV_ERROR(format_string_t<Args...> fmt, Args &&...args)
{
    CREATE_STATIC();
    spdClass.file_logger->set_level(spdlog::level::err);
    spdClass.my_logger->set_level(spdlog::level::err);
    spdClass.file_logger->error(fmt, std::forward<Args>(args)...);
    spdClass.my_logger->error(fmt, std::forward<Args>(args)...);
}

template <typename... Args>
inline void NAV_PASS(format_string_t<Args...> fmt, Args &&...args)
{
    CREATE_STATIC();
    spdClass.file_logger->set_level(spdlog::level::info);
    spdClass.my_logger->set_level(spdlog::level::info);
    spdClass.file_logger->info(fmt, std::forward<Args>(args)...);
    spdClass.my_logger->info(fmt, std::forward<Args>(args)...);
}


#ifdef NDEBUG
#define DEBUG_NAV_WARNING(msg) ((void)0)
#define DEBUG_NAV_ERROR(msg) ((void)0)
#define DEBUG_NAV_HIGHLIGHT(msg) ((void)0)
#define DEBUG_NAV_INFO(msg) ((void)0)
#define DEBUG_NAV_PASS(msg) ((void)0)
#else
template <typename... Args>
inline void DEBUG_NAV_HIGHLIGHT(format_string_t<Args...> fmt, Args &&...args)
{
    CREATE_STATIC();
    spdClass.file_logger->set_level(spdlog::level::critical);
    spdClass.my_logger->set_level(spdlog::level::critical);
    spdClass.file_logger->critical(fmt, std::forward<Args>(args)...);
    spdClass.my_logger->critical(fmt, std::forward<Args>(args)...);
}

template <typename... Args>
inline void DEBUG_NAV_WARNING(format_string_t<Args...> fmt, Args &&...args)
{
    CREATE_STATIC();
    spdClass.file_logger->set_level(spdlog::level::warn);
    spdClass.my_logger->set_level(spdlog::level::warn);
    spdClass.file_logger->warn(fmt, std::forward<Args>(args)...);
    spdClass.my_logger->warn(fmt, std::forward<Args>(args)...);
}

template <typename... Args>
inline void DEBUG_NAV_INFO(format_string_t<Args...> fmt, Args &&...args)
{
    // 获取当前 ROS 节点名称
    const char *node_name = ros::this_node::getName().c_str();
    if (!node_name)
    {
        node_name = "Unknown_Node";
    }

    CREATE_STATIC();
    spdClass.file_logger->set_level(spdlog::level::trace);
    spdClass.my_logger->set_level(spdlog::level::trace);
    spdClass.file_logger->trace(fmt, std::forward<Args>(args)...);
    spdClass.my_logger->trace(fmt, std::forward<Args>(args)...);
}

template <typename... Args>
inline void DEBUG_NAV_ERROR(format_string_t<Args...> fmt, Args &&...args)
{
    CREATE_STATIC();
    spdClass.file_logger->set_level(spdlog::level::err);
    spdClass.my_logger->set_level(spdlog::level::err);
    spdClass.file_logger->error(fmt, std::forward<Args>(args)...);
    spdClass.my_logger->error(fmt, std::forward<Args>(args)...);
}

template <typename... Args>
inline void DEBUG_NAV_PASS(format_string_t<Args...> fmt, Args &&...args)
{
    CREATE_STATIC();
    spdClass.file_logger->set_level(spdlog::level::info);
    spdClass.my_logger->set_level(spdlog::level::info);
    spdClass.file_logger->info(fmt, std::forward<Args>(args)...);
    spdClass.my_logger->info(fmt, std::forward<Args>(args)...);
}
#endif

#endif // SPD_LOGGING_H
#else
#error "nav_logging.h is already included, please do not include both spd_logging.h and nav_logging.h"
#endif // NAV_LOGGING_H