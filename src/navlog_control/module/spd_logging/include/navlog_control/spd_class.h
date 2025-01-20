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
    std::string ros_node_name = ros::this_node::getName();
    spd_class()
    {
        file_logger = spdlog::rotating_logger_mt(ros_node_name, navlog_pack + "/log_files/" + ros_node_name + ".txt", 1024 * 100000, 0);
        file_logger->set_pattern("[%Y-%m-%d %H:%M:%S][%n][%l] %v");


        my_logger= std::make_shared<spdlog::logger>("mylogger");//创建logger
        my_logger->set_pattern("[%Y-%m-%d %H:%M:%S][%n][%l] %v");//logger_1、设置格式
    }
    ~spd_class()
    {
        file_logger->flush();
    }
    std::shared_ptr<spdlog::logger> file_logger;
    std::shared_ptr<spdlog::logger> my_logger;
};

inline spd_class spdClass;