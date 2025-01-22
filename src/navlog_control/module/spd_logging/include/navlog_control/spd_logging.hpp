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
#include "spdlog/spdlog.h"
#include <functional>
#include <ros/this_node.h>

template <typename... Args>
using format_string_t = fmt::format_string<Args...>;

namespace spdlog_nav {
#define INIT_NAVLOG_ROS()                                  \
    do                                                     \
    {                                                      \
        std::string node_name = ros::this_node::getName(); \
        spd_class = spdClass(std::move(node_name));        \
    } while (false)

#define CREATE_STATIC()   \
    do                    \
    {                     \
        static int i = 1; \
        if (i++ < 30)     \
        {                 \
            return;       \
        }                 \
        i = 1;            \
    } while (false)

#define CREATE_STATIC_DEBUG() \
    do                        \
    {                         \
        static int i = 1;     \
        if (i++ < 150)        \
        {                     \
            return;           \
        }                     \
        i = 1;                \
    } while (false)

    template <typename... Args>
    inline void NAV_HIGHLIGHT(format_string_t<Args...> fmt, Args &&...args)
    {
        CREATE_STATIC();
        spd_class.file_logger->set_level(spdlog::level::critical);
        spd_class.my_logger->set_level(spdlog::level::critical);
        spd_class.file_logger->critical(fmt, std::forward<Args>(args)...);
        spd_class.my_logger->critical(fmt, std::forward<Args>(args)...);
    }

    template <typename... Args>
    inline void NAV_WARN(format_string_t<Args...> fmt, Args &&...args)
    {
        CREATE_STATIC();
        spd_class.file_logger->set_level(spdlog::level::warn);
        spd_class.my_logger->set_level(spdlog::level::warn);
        spd_class.file_logger->warn(fmt, std::forward<Args>(args)...);
        spd_class.my_logger->warn(fmt, std::forward<Args>(args)...);
    }

    template <typename... Args>
    inline void NAV_INFO(format_string_t<Args...> fmt, Args &&...args)
    {
        CREATE_STATIC();
        spd_class.file_logger->set_level(spdlog::level::trace);
        spd_class.my_logger->set_level(spdlog::level::trace);
        spd_class.file_logger->trace(fmt, std::forward<Args>(args)...);
        spd_class.my_logger->trace(fmt, std::forward<Args>(args)...);
    }

    template <typename... Args>
    inline void NAV_ERROR(format_string_t<Args...> fmt, Args &&...args)
    {
        CREATE_STATIC();
        spd_class.file_logger->set_level(spdlog::level::err);
        spd_class.my_logger->set_level(spdlog::level::err);
        spd_class.file_logger->error(fmt, std::forward<Args>(args)...);
        spd_class.my_logger->error(fmt, std::forward<Args>(args)...);
    }

    template <typename... Args>
    inline void NAV_PASS(format_string_t<Args...> fmt, Args &&...args)
    {
        CREATE_STATIC();
        spd_class.file_logger->set_level(spdlog::level::info);
        spd_class.my_logger->set_level(spdlog::level::info);
        spd_class.file_logger->info(fmt, std::forward<Args>(args)...);
        spd_class.my_logger->info(fmt, std::forward<Args>(args)...);
    }

    /*
     * @brief 单次打印函数
     */
    template <typename... Args>
    inline void NAV_HIGHLIGHT_ONCE(format_string_t<Args...> fmt, Args &&...args)
    {
        spd_class.file_logger->set_level(spdlog::level::critical);
        spd_class.my_logger->set_level(spdlog::level::critical);
        spd_class.file_logger->critical(fmt, std::forward<Args>(args)...);
        spd_class.my_logger->critical(fmt, std::forward<Args>(args)...);
    }

    template <typename... Args>
    inline void NAV_WARN_ONCE(format_string_t<Args...> fmt, Args &&...args)
    {
        spd_class.file_logger->set_level(spdlog::level::warn);
        spd_class.my_logger->set_level(spdlog::level::warn);
        spd_class.file_logger->warn(fmt, std::forward<Args>(args)...);
        spd_class.my_logger->warn(fmt, std::forward<Args>(args)...);
    }

    template <typename... Args>
    inline void NAV_INFO_ONCE(format_string_t<Args...> fmt, Args &&...args)
    {
        spd_class.file_logger->set_level(spdlog::level::trace);
        spd_class.my_logger->set_level(spdlog::level::trace);
        spd_class.file_logger->trace(fmt, std::forward<Args>(args)...);
        spd_class.my_logger->trace(fmt, std::forward<Args>(args)...);
    }

    template <typename... Args>
    inline void NAV_ERROR_ONCE(format_string_t<Args...> fmt, Args &&...args)
    {
        spd_class.file_logger->set_level(spdlog::level::err);
        spd_class.my_logger->set_level(spdlog::level::err);
        spd_class.file_logger->error(fmt, std::forward<Args>(args)...);
        spd_class.my_logger->error(fmt, std::forward<Args>(args)...);
    }

    template <typename... Args>
    inline void NAV_PASS_ONCE(format_string_t<Args...> fmt, Args &&...args)
    {
        spd_class.file_logger->set_level(spdlog::level::info);
        spd_class.my_logger->set_level(spdlog::level::info);
        spd_class.file_logger->info(fmt, std::forward<Args>(args)...);
        spd_class.my_logger->info(fmt, std::forward<Args>(args)...);
    }

#ifdef NDEBUG
#define DEBUG_NAV_WARN(msg...) ((void)0)
#define DEBUG_NAV_ERROR(msg...) ((void)0)
#define DEBUG_NAV_HIGHLIGHT(msg...) ((void)0)
#define DEBUG_NAV_INFO(msg...) ((void)0)
#define DEBUG_NAV_PASS(msg...) ((void)0)
#else
    template <typename... Args>
    inline void DEBUG_NAV_HIGHLIGHT(format_string_t<Args...> fmt, Args &&...args)
    {
        CREATE_STATIC_DEBUG();
        spd_class.file_logger->set_level(spdlog::level::critical);
        spd_class.my_logger->set_level(spdlog::level::critical);
        spd_class.file_logger->critical(fmt, std::forward<Args>(args)...);
        spd_class.my_logger->critical(fmt, std::forward<Args>(args)...);
    }

    template <typename... Args>
    inline void DEBUG_NAV_WARN(format_string_t<Args...> fmt, Args &&...args)
    {
        CREATE_STATIC_DEBUG();
        spd_class.file_logger->set_level(spdlog::level::warn);
        spd_class.my_logger->set_level(spdlog::level::warn);
        spd_class.file_logger->warn(fmt, std::forward<Args>(args)...);
        spd_class.my_logger->warn(fmt, std::forward<Args>(args)...);
    }

    template <typename... Args>
    inline void DEBUG_NAV_INFO(format_string_t<Args...> fmt, Args &&...args)
    {
        CREATE_STATIC_DEBUG();
        spd_class.file_logger->set_level(spdlog::level::trace);
        spd_class.my_logger->set_level(spdlog::level::trace);
        spd_class.file_logger->trace(fmt, std::forward<Args>(args)...);
        spd_class.my_logger->trace(fmt, std::forward<Args>(args)...);
    }

    template <typename... Args>
    inline void DEBUG_NAV_ERROR(format_string_t<Args...> fmt, Args &&...args)
    {
        CREATE_STATIC_DEBUG();
        spd_class.file_logger->set_level(spdlog::level::err);
        spd_class.my_logger->set_level(spdlog::level::err);
        spd_class.file_logger->error(fmt, std::forward<Args>(args)...);
        spd_class.my_logger->error(fmt, std::forward<Args>(args)...);
    }

    template <typename... Args>
    inline void DEBUG_NAV_PASS(format_string_t<Args...> fmt, Args &&...args)
    {
        CREATE_STATIC_DEBUG();
        spd_class.file_logger->set_level(spdlog::level::info);
        spd_class.my_logger->set_level(spdlog::level::info);
        spd_class.file_logger->info(fmt, std::forward<Args>(args)...);
        spd_class.my_logger->info(fmt, std::forward<Args>(args)...);
    }
#endif

#ifdef NDEBUG
    template <typename... Args>
    inline void RELEASE_NAV_HIGHLIGHT(format_string_t<Args...> fmt, Args &&...args)
    {
        CREATE_STATIC();
        spd_class.file_logger->set_level(spdlog::level::critical);
        spd_class.my_logger->set_level(spdlog::level::critical);
        spd_class.file_logger->critical(fmt, std::forward<Args>(args)...);
        spd_class.my_logger->critical(fmt, std::forward<Args>(args)...);
    }

    template <typename... Args>
    inline void RELEASE_NAV_WARN(format_string_t<Args...> fmt, Args &&...args)
    {
        CREATE_STATIC();
        spd_class.file_logger->set_level(spdlog::level::warn);
        spd_class.my_logger->set_level(spdlog::level::warn);
        spd_class.file_logger->warn(fmt, std::forward<Args>(args)...);
        spd_class.my_logger->warn(fmt, std::forward<Args>(args)...);
    }

    template <typename... Args>
    inline void RELEASE_NAV_INFO(format_string_t<Args...> fmt, Args &&...args)
    {
        CREATE_STATIC();
        spd_class.file_logger->set_level(spdlog::level::trace);
        spd_class.my_logger->set_level(spdlog::level::trace);
        spd_class.file_logger->trace(fmt, std::forward<Args>(args)...);
        spd_class.my_logger->trace(fmt, std::forward<Args>(args)...);
    }

    template <typename... Args>
    inline void RELEASE_NAV_ERROR(format_string_t<Args...> fmt, Args &&...args)
    {
        CREATE_STATIC();
        spd_class.file_logger->set_level(spdlog::level::err);
        spd_class.my_logger->set_level(spdlog::level::err);
        spd_class.file_logger->error(fmt, std::forward<Args>(args)...);
        spd_class.my_logger->error(fmt, std::forward<Args>(args)...);
    }

    template <typename... Args>
    inline void RELEASE_NAV_PASS(format_string_t<Args...> fmt, Args &&...args)
    {
        CREATE_STATIC();
        spd_class.file_logger->set_level(spdlog::level::info);
        spd_class.my_logger->set_level(spdlog::level::info);
        spd_class.file_logger->info(fmt, std::forward<Args>(args)...);
        spd_class.my_logger->info(fmt, std::forward<Args>(args)...);
    }
#else
    #define RELEASE_NAV_WARN(msg...) ((void)0)
    #define RELEASE_NAV_ERROR(msg...) ((void)0)
    #define RELEASE_NAV_HIGHLIGHT(msg...) ((void)0)
    #define RELEASE_NAV_INFO(msg...) ((void)0)
    #define RELEASE_NAV_PASS(msg...) ((void)0)
#endif
} // namespace spdlog_nav

using namespace spdlog_nav;

#endif // SPD_LOGGING_H
#else
#error "nav_logging.h is already included, please do not include both spd_logging.h and nav_logging.h"
#endif // NAV_LOGGING_H