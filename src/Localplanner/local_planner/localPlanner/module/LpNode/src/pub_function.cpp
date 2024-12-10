#include "LpNode.h"

void LpNode::pub_allFreePath()
{
    // ROS_WARN("Publishing free paths.");
    // freePaths 则是所有 clearPathList[i] <= pointPerPathThre 的路径，即不存在障碍物的路径
    freePaths->clear();
    for (int i = 0; i < 36 * pathNum; i++)
    {
        // 遍历360度朝向
        int rotDir = int(i / pathNum);
        float rotAng = (10.0 * rotDir - 180.0) * PI / 180;
        float rotDeg = 10.0 * rotDir;
        if (rotDeg > 180.0)
            rotDeg -= 360.0;
        float angDiff = fabs(joyDir - (10.0 * rotDir - 180.0));
        if (angDiff > 180.0)
        {
            angDiff = 360.0 - angDiff;
        }
        // 相对目标点之间的规划扇形范围
        if ((angDiff > lctlPtr->get_params().dirThre) || !((rotAng * 180.0 / PI > minObsAngCW && rotAng * 180.0 / PI < minObsAngCCW) || !checkRotObstacle))
        {
            continue;
        }

        // 无障碍物路径下的所有距离内采样点加入freePaths
        if (clearPathList[i] <= lctlPtr->get_params().pointPerPathThre)
        {
            int freePathLength = paths[i % pathNum]->points.size();
            for (int j = 0; j < freePathLength; j++)
            {
                point = paths[i % pathNum]->points[j];

                float x = point.x;
                float y = point.y;
                float z = point.z;

                float dis = sqrt(x * x + y * y);
                // 点云往里收缩pathScale，可行路就可以往外扩张pathScale
                if (dis <= pathRange / pathScale && (dis <= (relativeGoalDis + actual_goalClearRange) || !lctlPtr->get_params().pathCropByGoal) && (dis <= (relativeGoalDis_global + lctlPtr->get_params().goalClearRange_global)))
                {
                    // 转车头系
                    point.x = (cos(rotAng) * x - sin(rotAng) * y) / pathScale;
                    point.y = (sin(rotAng) * x + cos(rotAng) * y) / pathScale;
                    point.z = z;
                    point.intensity = 1.0;
                    freePaths->push_back(point);
                }
            }
        }
    }

    sensor_msgs::PointCloud2 freePaths2;
    pcl::toROSMsg(*freePaths, freePaths2);
    freePaths2.header.stamp = ros::Time().fromSec(odomTime);
    freePaths2.header.frame_id = robotFrame;
    pubFreePaths.publish(freePaths2);
}

void LpNode::pubPath_fromStartPaths(int &selectedGroupID)
{
    int rotDir = int(selectedGroupID / groupNum);
    rotAng = (10.0 * rotDir - 180.0) * PI / 180;
    nav_msgs::Path path;
    selectedGroupID = selectedGroupID % groupNum;                        // 转为路径组的方向
    int selectedPathLength = startPaths[selectedGroupID]->points.size(); // 预设路径采样在这个方向上的路点采样点数
    path.poses.resize(selectedPathLength);
    int i = 0;
    for (i = 0; i < selectedPathLength; i++) // 遍历对应的路径组
    {
        float x = startPaths[selectedGroupID]->points[i].x;
        float y = startPaths[selectedGroupID]->points[i].y;
        float z = startPaths[selectedGroupID]->points[i].z;
        float dis = sqrt(x * x + y * y);
        // 局部路径的范围，小于等于预设路径的范围。因为除此之外，还要满足小于设定的pathRange
        if (dis <= pathRange / pathScale && dis <= relativeGoalDis)
        {
            path.poses[i].header.frame_id = robotFrame;
            path.poses[i].header.stamp = ros::Time().now();
            path.poses[i].pose.position.x = (cos(rotAng) * x - sin(rotAng) * y) / pathScale; // 叠加上路径最相对车头的方向，目标以车头为系
            path.poses[i].pose.position.y = (sin(rotAng) * x + cos(rotAng) * y) / pathScale;
            path.poses[i].pose.position.z = z;
            path.poses[i].pose.orientation.w = 1;
        }
        else
        {
            path.poses.resize(i); // 路径resize不会清空已有路径点
            break;
        }
    }
    path.header.stamp = ros::Time().now();
    path.header.frame_id = robotFrame;
    pubPath.publish(path);

    // 转成世界坐标系后重新发布，方便可视化
    for (int j = 0; j < i; j++)
    {
        float x = path.poses[j].pose.position.x;
        float y = path.poses[j].pose.position.y;
        path.poses[j].pose.position.x = cos(vehicleYaw) * x - sin(vehicleYaw) * y + vehicleX;
        path.poses[j].pose.position.y = sin(vehicleYaw) * x + cos(vehicleYaw) * y + vehicleY;
        path.poses[j].pose.position.z = vehicleZ;
        path.poses[j].header.frame_id = "map";
        path.poses[j].header.stamp = ros::Time().now();
    }
    path.header.stamp = ros::Time().now();
    path.header.frame_id = "map";
    pubGlobalPath.publish(path);
}

void LpNode::pub_Map()
{
    if (lctlPtr->get_params().use_map)
    {
        pubMap.publish(terrainMapRecord);
    }
    else
    {
        sensor_msgs::PointCloud2 temp;
        temp.header.stamp = ros::Time().now();
        temp.header.frame_id = "map";
        pubMap.publish(temp);
    }
}
