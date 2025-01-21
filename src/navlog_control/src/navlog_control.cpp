// #define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_DEBUG
#include "navlog_control/spd_logging.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "navlog_control");
    ros::NodeHandle nh;
    ROS_INFO("navlog_control init!");
    NAV_HIGHLIGHT("NAV_HIGHLIGHT_init!");
    NAV_DEBUG("NAV_DEBUG_init!");
    NAV_INFO("NAV_INFO_init!");
    while (ros::ok())
    {
        static int i = 0;
        NAV_HIGHLIGHT("NAV_HIGHLIGHT111! {}", i++);
        NAV_HIGHLIGHT("NAV_HIGHLIGHT222! {}", i);
    }
    return 0;
}