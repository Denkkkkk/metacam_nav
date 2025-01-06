/**
 * @file serial_logging.h
 * @author zhaoxi (535394140@qq.com)
 *         钟华
 *         ldq
 * @brief  日志打印管理模块
 * @version 2.0
 * @date 2022-11-03
 * @update 2024-12-27
 * @copyright Copyright SCUT RobotLab(c) 2022
 *
 */
#ifndef SPD_LOGGING_H
#ifndef NAV_LOGGING_H
#define NAV_LOGGING_H
#include <cstdio> // for printf
#include <cstring>
#include <ctime>
#include <iostream>
#include <ros/ros.h>
#include <unistd.h>

//! @addtogroup nav_logging标记文档组开始
//! @{
#ifdef NDEBUG
#define DEBUG_NAV_WARNING(msg) ((void)0)
#define DEBUG_NAV_ERROR(msg) ((void)0)
#define DEBUG_NAV_HIGHLIGHT(msg) ((void)0)
#define DEBUG_NAV_INFO(msg) ((void)0)
#define DEBUG_NAV_PASS(msg) ((void)0)
#else
#define DEBUG_NAV_WARNING(msg) NAV_WARNING(msg)
#define DEBUG_NAV_ERROR(msg) NAV_ERROR(msg)
#define DEBUG_NAV_HIGHLIGHT(msg) NAV_HIGHLIGHT(msg)
#define DEBUG_NAV_INFO(msg) NAV_INFO(msg)
#define DEBUG_NAV_PASS(msg) NAV_PASS(msg)
#endif

// 35m 品红
#define NAV_HIGHLIGHT(msg...)                                           \
    do                                                                  \
    {                                                                   \
        /* 获取当前时间 */                                              \
        time_t rawtime;                                                 \
        struct tm *timeinfo;                                            \
        char timeStr[20];                                               \
        time(&rawtime);                                                 \
        timeinfo = localtime(&rawtime);                                 \
        strftime(timeStr, sizeof(timeStr), "%m-%d %H:%M:%S", timeinfo); \
                                                                        \
        /* 获取当前 ROS 节点名称 */                                     \
        const char *node_name = ros::this_node::getName().c_str();      \
        if (!node_name)                                                 \
        {                                                               \
            node_name = "Unknown_Node";                                 \
        }                                                               \
                                                                        \
        /* 打印日志消息 */                                              \
        printf("\033[35m[%s] [%s] ", node_name, timeStr);               \
        printf(msg);                                                    \
        printf("\033[0m\n");                                            \
    } while (false)

// 33m 黄色
#define NAV_WARNING(msg...)                                             \
    do                                                                  \
    {                                                                   \
        /* 获取当前时间 */                                              \
        time_t rawtime;                                                 \
        struct tm *timeinfo;                                            \
        char timeStr[20];                                               \
        time(&rawtime);                                                 \
        timeinfo = localtime(&rawtime);                                 \
        strftime(timeStr, sizeof(timeStr), "%m-%d %H:%M:%S", timeinfo); \
                                                                        \
        /* 获取当前 ROS 节点名称 */                                     \
        const char *node_name = ros::this_node::getName().c_str();      \
        if (!node_name)                                                 \
        {                                                               \
            node_name = "Unknown_Node";                                 \
        }                                                               \
                                                                        \
        /* 打印日志消息 */                                              \
        printf("\033[33m[%s] [%s] ", node_name, timeStr);               \
        printf(msg);                                                    \
        printf("\033[0m\n");                                            \
    } while (false)

// 32m 绿色
#define NAV_PASS(msg...)                                                \
    do                                                                  \
    {                                                                   \
        /* 获取当前时间 */                                              \
        time_t rawtime;                                                 \
        struct tm *timeinfo;                                            \
        char timeStr[20];                                               \
        time(&rawtime);                                                 \
        timeinfo = localtime(&rawtime);                                 \
        strftime(timeStr, sizeof(timeStr), "%m-%d %H:%M:%S", timeinfo); \
                                                                        \
        /* 获取当前 ROS 节点名称 */                                     \
        const char *node_name = ros::this_node::getName().c_str();      \
        if (!node_name)                                                 \
        {                                                               \
            node_name = "Unknown_Node";                                 \
        }                                                               \
                                                                        \
        /* 打印日志消息 */                                              \
        printf("\033[32m[%s] [%s] ", node_name, timeStr);               \
        printf(msg);                                                    \
        printf("\033[0m\n");                                            \
    } while (false)

// 31m 红色
#define NAV_ERROR(msg...)                                               \
    do                                                                  \
    {                                                                   \
        /* 获取当前时间 */                                              \
        time_t rawtime;                                                 \
        struct tm *timeinfo;                                            \
        char timeStr[20];                                               \
        time(&rawtime);                                                 \
        timeinfo = localtime(&rawtime);                                 \
        strftime(timeStr, sizeof(timeStr), "%m-%d %H:%M:%S", timeinfo); \
                                                                        \
        /* 获取当前 ROS 节点名称 */                                     \
        const char *node_name = ros::this_node::getName().c_str();      \
        if (!node_name)                                                 \
        {                                                               \
            node_name = "Unknown_Node";                                 \
        }                                                               \
                                                                        \
        /* 打印日志消息 */                                              \
        printf("\033[31m[%s] [%s] ", node_name, timeStr);               \
        printf(msg);                                                    \
        printf("\033[0m\n");                                            \
    } while (false)

#define NAV_INFO(msg...)                                                \
    do                                                                  \
    {                                                                   \
        /* 获取当前时间 */                                              \
        time_t rawtime;                                                 \
        struct tm *timeinfo;                                            \
        char timeStr[20];                                               \
        time(&rawtime);                                                 \
        timeinfo = localtime(&rawtime);                                 \
        strftime(timeStr, sizeof(timeStr), "%m-%d %H:%M:%S", timeinfo); \
                                                                        \
        /* 获取当前 ROS 节点名称 */                                     \
        const char *node_name = ros::this_node::getName().c_str();      \
        if (!node_name)                                                 \
        {                                                               \
            node_name = "Unknown_Node";                                 \
        }                                                               \
                                                                        \
        /* 打印日志消息 */                                              \
        printf(node_name, timeStr);                                     \
        printf(msg);                                                    \
        printf("\n");                                                   \
    } while (false)

//! @} nav_logging
#endif // NAV_LOGGING_H
#else
#error "spd_logging.h is already included, please do not include both spd_logging.h and nav_logging.h"
#endif // SPD_LOGGING_H