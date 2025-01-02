#ifndef NAV_LOGGING_H
#ifndef SPD_LOGGING_H
#define SPD_LOGGING_H
#include "spdlog/spdlog.h"
#include "ros/ros.h"

void logWithColor(const char* color, const char* node_name, const char* timeStr, const char* msg);

void NAV_HIGHLIGHT(const char* msg);
void NAV_WARNING(const char* msg);
void NAV_PASS(const char* msg);
void NAV_ERROR(const char* msg);
void NAV_INFO(const char* msg);

#endif // SPD_LOGGING_H
#else
#error "nav_logging.h is already included, please do not include both spd_logging.h and nav_logging.h"
#endif // NAV_LOGGING_H