#include "navlog_control/nav_logging.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "navlog_control");
    ros::NodeHandle nh;
    for (int i = 0; i < 10; i++)
    {
//        NAV_INFO("This is a NAV_INFO message %d", i);
//        NAV_WARNING("This is a NAV_WARNING message %s", "test");
//        NAV_ERROR("This is a NAV_ERROR message %f", i * 0.1);
//        NAV_HIGHLIGHT("This is a NAV_HIGHLIGHT message %d", i);
          NAV_HIGHLIGHT("This is a NAV_HIGHLIGHT message %d", 1);
    }
    ROS_INFO( "Hello, world!" );

    return 0;
}