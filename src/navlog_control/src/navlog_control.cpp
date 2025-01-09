#define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_INFO
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
    spdlog::details::log_msg my_msg("mylogger", spdlog::level::info, "Hello World!");
    spdClass.info_sink->log(my_msg); // sink_3、sink输出
}