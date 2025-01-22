#define SPDLOG_COLOR_MODE SPDLOG_COLOR_MODE_AUTO
#include "spdlog/sinks/basic_file_sink.h"
#include "spdlog/spdlog.h"
#include <iostream>
#include <spdlog/sinks/rotating_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h> // 引入终端彩色输出 sink
#include <ros/package.h>

class spdClass
{

private:
    std::string node_name_;
    std::shared_ptr<spdlog::sinks::stdout_color_sink_mt> console_sink;
public:
    std::string navlog_pack = ros::package::getPath("navlog_control");

    spdClass()
    {
        file_logger = spdlog::rotating_logger_mt("navlog_control",navlog_pack + "/log_files/" + "common" + ".log", 1024 * 100000, 0);
        file_logger->set_pattern("[%Y-%m-%d %H:%M:%S] %v");

        console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>(); // 创建控制台输出 sink
        console_sink->set_pattern("[%^%Y-%m-%d %H:%M:%S] %v%$"); // 设置输出格式
        my_logger = std::make_shared<spdlog::logger>("mylogger",console_sink); // 创建logger
    }

    // explicit防止隐式参数类型转换
    explicit spdClass(std::string&& node_name)
    {
        node_name_ = std::move(node_name);
        file_logger = spdlog::rotating_logger_mt(node_name_, navlog_pack + "/log_files/" + node_name_ + ".log", 1024 * 100000, 0);
        file_logger->set_pattern("[%Y-%m-%d %H:%M:%S][%n] %v");

        console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>(); // 创建控制台输出 sink
        console_sink->set_pattern("[%^%Y-%m-%d %H:%M:%S][%n] %v%$"); // 设置输出格式
        my_logger = std::make_shared<spdlog::logger>(node_name_,console_sink); // 创建logger
    }

    ~spdClass()
    {
        file_logger->flush();
    }
    std::shared_ptr<spdlog::logger> file_logger;
    std::shared_ptr<spdlog::logger> my_logger;
};

inline spdClass spd_class;