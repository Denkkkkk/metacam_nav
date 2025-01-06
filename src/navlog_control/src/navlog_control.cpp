#define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_INFO
#include "navlog_text/spd_logging.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "navlog_control");
    ros::NodeHandle nh;
    for (int i = 0; i < 10; i++)
    {
          spdlog::error("Welcome to spd_trace! {}",i);
          NAV_HIGHLIGHT("Welcome to NAV_HIGHLIGHT! {}",i);
    }
    return 0;
}