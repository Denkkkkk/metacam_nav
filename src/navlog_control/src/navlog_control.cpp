#define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_DEBUG
#include "navlog_control/spd_logging.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "navlog_control");
    ros::NodeHandle nh;
    while (ros::ok())
    {
        static int i = 0;
        NAV_HIGHLIGHT("NAV_HIGHLIGHT111! {}", i++);
        NAV_HIGHLIGHT("NAV_HIGHLIGHT222! {}", i);
    }
    return 0;
}