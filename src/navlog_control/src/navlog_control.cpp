#include "navlog_control/spd_logging.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "navlog_control");
    ros::NodeHandle nh;
    ros::Rate rate(10);
    while (ros::ok())
    {
        static int i = 0;
        NAV_WARNING("NAV_WARNING1!");
        NAV_PASS("NAV_PASS2! {}", i++);
        NAV_INFO("NAV_INFO3!");
        NAV_HIGHLIGHT("NAV_HIGHLIGHT4!");
        NAV_ERROR("NAV_ERROR5!");

        DEBUG_NAV_WARNING("DEBUG_NAV_WARNING1!");
        DEBUG_NAV_PASS("DEBUG_NAV_PASS2!");
        DEBUG_NAV_INFO("DEBUG_NAV_INFO3!");
        DEBUG_NAV_HIGHLIGHT("DEBUG_NAV_HIGHLIGHT4!");
        DEBUG_NAV_ERROR("DEBUG_NAV_ERROR5!");
        rate.sleep();
    }
    return 0;
}