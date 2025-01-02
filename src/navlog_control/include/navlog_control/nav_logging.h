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

#include <cstdio>
#include <cstring>
#include <ctime>
#include <iostream>
#include <ros/ros.h>
#include <unistd.h>

//! @addtogroup serial_logging
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
void logWithColor(const char* color, const char* node_name, const char* timeStr, const char* msg);
void NAV_HIGHLIGHT(const char* msg);
void NAV_WARNING(const char* msg);
void NAV_PASS(const char* msg);
void NAV_ERROR(const char* msg);
void NAV_INFO(const char* msg);