#include "spdlog/sinks/basic_file_sink.h"
#include "spdlog/spdlog.h"
#include <filesystem>
#include <iostream>
#include <spdlog/sinks/rotating_file_sink.h>

#include <ros/package.h>
#include <ros/this_node.h>

namespace fs = std::filesystem;

class spd_class
{

private:
public:
    std::string navlog_pack = ros::package::getPath("navlog_control");

    spd_class()
    {
        // 获取当前源文件名（去除路径）
        std::string current_file_name = __FILE__;
        // 提取文件名（去除路径部分）
        size_t last_slash_pos = current_file_name.find_last_of("/\\");
        if (last_slash_pos != std::string::npos) {
            current_file_name = current_file_name.substr(last_slash_pos + 1);
        }

        file_logger = spdlog::rotating_logger_mt(,navlog_pack + "/log_files/" + current_file_name + ".log", 1024 * 100000, 0);
        file_logger->set_pattern("[%Y-%m-%d %H:%M:%S][%l] %v");

        my_logger = std::make_shared<spdlog::logger>("mylogger"); // 创建logger
        my_logger->set_pattern("[%Y-%m-%d %H:%M:%S][%l] %v"); // logger_1、设置格式
    }

    // explicit防止隐式参数类型转换
    explicit spd_class(ros::NodeHandle &nh)
    {
        std::string ros_node_name = nh.getName();
        file_logger = spdlog::rotating_logger_mt(ros_node_name, navlog_pack + "/log_files/" + ros_node_name + ".log", 1024 * 100000, 0);
        file_logger->set_pattern("[%Y-%m-%d %H:%M:%S][%n][%l] %v");
        my_logger = std::make_shared<spdlog::logger>("mylogger"); // 创建logger
        my_logger->set_pattern("[%Y-%m-%d %H:%M:%S][%n][%l] %v"); // logger_1、设置格式
    }

    ~spd_class()
    {
        file_logger->flush();
    }
    std::shared_ptr<spdlog::logger> file_logger;
    std::shared_ptr<spdlog::logger> my_logger;
};

inline spd_class spdClass;