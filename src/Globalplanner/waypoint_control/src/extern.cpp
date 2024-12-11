#include "extern.h"

double vehicleX = 0, vehicleY = 0, vehicleZ = 0, vehicleYaw = 0;

geometry_msgs::PoseStamped point_last; // 上一个给点
nav_msgs::Path goal_path;
