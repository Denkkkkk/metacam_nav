#define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_DEBUG
#include "navlog_control/spd_logging.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "navlog_control");
    ros::NodeHandle nh;
    for (int i = 0; i < 10; i++)
    {
        //   spdlog::error("Welcome to spd_trace! {}",i);
        NAV_HIGHLIGHT("Welcome to NAV_HIGHLIGHT! {}", i);
    }

    std::cout << "cout Hello World!" << std::endl;

    auto my_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>("./fileName.txt"); // 创建sink指针
    my_sink->set_pattern("[%Y-%m-%d %H:%M:%S] %v");                                       // sink_1、设置格式
    my_sink->set_level(spdlog::level::info);                                              // sink_2、设置最低输出等级

    auto my_logger = std::make_shared<spdlog::logger>("mylogger", my_sink); // 创建logger

    spdlog::details::log_msg my_msg("mylogger", spdlog::level::info, "Hello World!");
    my_sink->log(my_msg); // sink_3、sink输出

    return 0;
}