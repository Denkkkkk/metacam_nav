#include "spdlog/sinks/basic_file_sink.h"
#include "spdlog/spdlog.h"
#include <filesystem>
#include <iostream>

namespace fs = std::filesystem;

class spd_class
{

private:
    /* data */

    /*func*/
    static void create_log_directory_if_not_exists(const std::string &dir_path);

public:
    spd_class()
    {
        // create_log_directory_if_not_exists("./navlog.txt");
        info_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>("/home/denk/skyland_innovation/metacam_nav/src/navlog_control/log_files/navlog.txt"); // 创建sink指针
        info_sink->set_pattern("[%Y-%m-%d %H:%M:%S] %v");                                                                                                     // sink_1、设置格式
        info_sink->set_level(spdlog::level::info);                                                                                                            // sink_2、设置最低输出等级
    }
    ~spd_class()
    {
        logger->flush();
        info_sink->flush();
    }
    std::shared_ptr<spdlog::logger> logger = spdlog::default_logger();
    std::shared_ptr<spdlog::sinks::basic_file_sink_mt> info_sink;


    auto my_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>("./fileName.txt");//创建sink指针
    my_sink->set_pattern("[%Y-%m-%d %H:%M:%S] %v");//sink_1、设置格式
    my_sink->set_level(spdlog::level::info);//sink_2、设置最低输出等级

    auto my_logger= std::make_shared<spdlog::logger>("mylogger", my_sink);//创建logger

    spdlog::details::log_msg my_msg("mylogger", spdlog::level::info, "Hello World!");
    my_sink->log(my_msg);//sink_3、sink输出
};

inline spd_class spdClass;