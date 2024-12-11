#include "midpoint.h"

MidPointMode::MidPointMode()
{
    _pubTurnPoint = _nh.advertise<geometry_msgs::PointStamped>("/turn_point", 5);
    load_param();
}

void MidPointMode::load_param()
{
    _nhPrivate.param("use_midplan", use_midplan, true);
    _nhPrivate.param("get_midPlanner_dis", get_midPlanner_dis, 0.2);
    _nhPrivate.param("need_midplan_dis", need_midplan_dis, 1.5);
    _nhPrivate.param("lenth_forward", _lenth_forward, 0.5);
    _nhPrivate.param("lenth_back", _lenth_back, 0.5);
    _nhPrivate.param("curvature_threshold", _curvature_threshold, 30.0);
    _nhPrivate.param("no_midPlanner_forward", _no_midPlanner_forward, 0.2);
    _nhPrivate.param("mid_point_back", _mid_point_back, 0.2);
    _nhPrivate.param("update_begin_dis", _update_begin_dis, 0.1);

    _nhPrivate.param("use_guide", use_guide, true);
    _nhPrivate.param("guide_dis", guide_dis, 0.5);
}

/**
 * @brief 转弯点计算和导航点选择
 *
 */
void MidPointMode::countMidPoint(bool &pathNew)
{
    int pathPointforward = 0;
    int pathPointback = 0;
    int pathPointID = 100000;
    int no_midPlanner_forwardID = 1;
    double endpathDis;
    if (use_midplan && pathNew)
    {
        int pathSize = goal_path.poses.size();
        if (pathSize >= 2)
        {
            // 最优路径的最后一个点到最新车体位置的X方向的距离
            float endpathDisX = goal_path.poses[pathSize - 1].pose.position.x - vehicleX;
            float endpathDisY = goal_path.poses[pathSize - 1].pose.position.y - vehicleY;
            endpathDis = sqrt(endpathDisX * endpathDisX + endpathDisY * endpathDisY);
        }
        if (pathSize < 2 || endpathDis < need_midplan_dis)
        {
            mid_point_exist = false;
            pathNew = false;
        }
        else
        {
            pathPointID = 1;
            // 找出在dis < no_midPlanner_forward内的点ID
            for (int i = 0; i < pathSize; i++)
            {
                float disX = goal_path.poses[i].pose.position.x - vehicleX;
                float disY = goal_path.poses[i].pose.position.y - vehicleY;
                float dis = sqrt(disX * disX + disY * disY);
                if (dis > _no_midPlanner_forward)
                {
                    no_midPlanner_forwardID = i;
                    break;
                }
            }
            pathPointID = no_midPlanner_forwardID + 1;
            while (pathPointID < pathSize - 1)
            {
                for (int i = pathPointID; i >= 0; i--)
                {
                    float disX = goal_path.poses[i].pose.position.x - goal_path.poses[pathPointID].pose.position.x;
                    float disY = goal_path.poses[i].pose.position.y - goal_path.poses[pathPointID].pose.position.y;
                    float dis = sqrt(disX * disX + disY * disY);
                    if (dis > _lenth_forward)
                    {
                        pathPointforward = i;
                        break;
                    }
                    if (i <= no_midPlanner_forwardID)
                    {
                        pathPointforward = -1;
                    }
                }
                if (pathPointforward == -1)
                {
                    pathPointID++;
                    continue;
                }
                for (int i = pathPointID; i < pathSize; i++)
                {
                    float disX = goal_path.poses[i].pose.position.x - goal_path.poses[pathPointID].pose.position.x;
                    float disY = goal_path.poses[i].pose.position.y - goal_path.poses[pathPointID].pose.position.y;
                    float dis = sqrt(disX * disX + disY * disY);
                    if (dis > _lenth_back)
                    {
                        pathPointback = i;
                        break;
                    }
                    if (i == pathSize - 1)
                    {
                        pathPointback = -1;
                    }
                }
                if (pathPointback == -1)
                {
                    pathPointID++;
                    mid_point_exist = false;
                    pathNew = false;
                    break; // 后面已经找到尽头，结束了
                }
                float curvature = 0;
                float yawforward = atan2(goal_path.poses[pathPointID].pose.position.y - goal_path.poses[pathPointforward].pose.position.y, goal_path.poses[pathPointID].pose.position.x - goal_path.poses[pathPointforward].pose.position.x);
                float yawback = atan2(goal_path.poses[pathPointback].pose.position.y - goal_path.poses[pathPointID].pose.position.y, goal_path.poses[pathPointback].pose.position.x - goal_path.poses[pathPointID].pose.position.x);
                curvature = yawback - yawforward;
                curvature = curvature * 180 / M_PI; // 统一转换为度
                if (abs(curvature) > _curvature_threshold)
                {
                    // 发布的点从转弯点延后 mid_point_back
                    for (int i = pathPointID; i < pathSize; i++)
                    {
                        float disX = goal_path.poses[i].pose.position.x - goal_path.poses[pathPointID].pose.position.x;
                        float disY = goal_path.poses[i].pose.position.y - goal_path.poses[pathPointID].pose.position.y;
                        float dis = sqrt(disX * disX + disY * disY);
                        if (dis > _mid_point_back)
                        {
                            pathPointID = i;
                            break;
                        }
                    }
                    mid_point_exist = true;
                    mid_point.pose.position.x = goal_path.poses[pathPointID].pose.position.x;
                    mid_point.pose.position.y = goal_path.poses[pathPointID].pose.position.y;
                    mid_point.pose.position.z = goal_path.poses[pathPointID].pose.position.z;
                    mid_point.pose.orientation = goal_path.poses[pathPointID].pose.orientation;
                    break;
                }
                pathPointID++;
            }
        }
    }
    if (!use_guide)
    {
        return;
    }

    int guide_id = -1;
    if (use_guide)
    {
        // 查看是否有引导点
        int pathSize = goal_path.poses.size();
        for (int i = 0; i < pathSize; i++)
        {
            float disX = goal_path.poses[i].pose.position.x - vehicleX;
            float disY = goal_path.poses[i].pose.position.y - vehicleY;
            float dis = sqrt(disX * disX + disY * disY);
            if (dis > guide_dis)
            {
                guide_id = i;
                _guide_exist = true;
                break;
            }
        }
        // ROS_ERROR("pathPointID: %d, guide_id: %d", pathPointID, guide_id);
        if (guide_id == -1)
        {
            _guide_exist = false;
        }
    }
    // 同时开启，中间点在前面必须优先
    if (use_guide && use_midplan || is_midplanning)
    {
        if (mid_point_exist && pathPointID < guide_id + 30)
        {
            double dis = sqrt(pow(mid_point.pose.position.x - point_last.pose.position.x, 2) + pow(mid_point.pose.position.y - point_last.pose.position.y, 2));
            if (dis > _update_begin_dis)
            {
                // 借助转弯点规划，或期间找到新的转弯点，更新一下
                is_midplanning = true;
                pub_guide = true;
                return;
            }
        }
        if (is_midplanning)
        {
            // 如果这时允许进来的话，正在借助转弯点规划，又找不到新的转弯点，不更新
            mid_point.pose.position.x = point_last.pose.position.x;
            mid_point.pose.position.y = point_last.pose.position.y;
            mid_point.pose.position.z = point_last.pose.position.z;
            mid_point.pose.orientation = point_last.pose.orientation;
            pub_guide = false;
            return;
        }
    }
    if (_guide_exist)
    {
        double dis = sqrt(pow(goal_path.poses[guide_id].pose.position.x - point_last.pose.position.x, 2) + pow(goal_path.poses[guide_id].pose.position.y - point_last.pose.position.y, 2));
        if (dis > _update_begin_dis)
        {
            mid_point.pose.position.x = goal_path.poses[guide_id].pose.position.x;
            mid_point.pose.position.y = goal_path.poses[guide_id].pose.position.y;
            mid_point.pose.position.z = goal_path.poses[guide_id].pose.position.z;
            mid_point.pose.orientation = goal_path.poses[guide_id].pose.orientation;
            mid_point_exist = true;
            pub_guide = true;
            return;
        }
    }
    pub_guide = false;
}

void MidPointMode::pub_turn_point()
{
    if (is_midplanning)
    {
        geometry_msgs::PointStamped turn_point;
        turn_point.point.x = mid_point.pose.position.x;
        turn_point.point.y = mid_point.pose.position.y;
        turn_point.point.z = mid_point.pose.position.z;
        turn_point.header.frame_id = "map";
        turn_point.header.stamp = ros::Time::now();
        _pubTurnPoint.publish(turn_point);
    }
    else
    {
        geometry_msgs::PointStamped turn_point;
        turn_point.point.x = 100;
        turn_point.point.y = 100;
        turn_point.point.z = 0;
        turn_point.header.frame_id = "map";
        turn_point.header.stamp = ros::Time::now();
        _pubTurnPoint.publish(turn_point);
    }
}