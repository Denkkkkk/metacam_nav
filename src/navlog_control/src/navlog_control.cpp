#include "navlog_control/nav_logging.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "navlog_control");
    ros::NodeHandle nh;
    for (int i = 0; i < 10; i++)
    {
        NAV_INFO("This is a ROS_INFO message %d", i);
        NAV_WARN("This is a ROS_WARN message %s", "test");
        NAV_ERROR("This is a ROS_ERROR message %f", i * 0.1);
        NAV_HIGHLIGHT("This is a ROS_FATAL message %d", i);
    }
    return 0;
}