#include "navlog_control/spd_logging.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "navlog_control");
    ros::NodeHandle nh;
    for (int i = 0; i < 10; i++)
    {
          spdlog::trace("Welcome to spd_trace!");
          NAV_HIGHLIGHT("Welcome to NAV_HIGHLIGHT!");
    }

    return 0;
}