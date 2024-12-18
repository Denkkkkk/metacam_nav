/**
 * @file LpNode.cpp
 * @author 李东权 (1327165187@qq.com)
 * @brief 局部规划器类实现
 * @version 1.0
 * @date 2024-01-12
 *
 * @copyright Copyright (c) 2024
 *
 */
#include "LpNode.h"

LpNode::LpNode() : terrainMapRecord_pcl(new pcl::PointCloud<pcl::PointXYZI>())
{
    nhPrivate.getParam("pathFolder", pathFolder);

    nhPrivate.getParam("goalX", goalX);
    nhPrivate.getParam("goalY", goalY);
    nhPrivate.getParam("usual_pcd_path", usual_pcd_path);
    nhPrivate.getParam("detourWei", detourWei);
    std::string ns;
    nhPrivate.getParam("ns", ns);
    if (ns != "")
    {
        ns += "/";
    }
    robotFrame = ns + "vehicle";
    lctlPtr = new ParamControl();

    load_pcd_map();
    // 全局目标点初始化到很远
    goal_point.pose.position.x = 100;
    goal_point.pose.position.y = 100;
    joySpeed = maxSpeed;

    // 回调函数，提取里程计信息，更新车体的位姿信息
    subOdometry = nh.subscribe<nav_msgs::Odometry>("/odom_interface", 5, &LpNode::odometryHandler, this);
    // 订阅地形点云/terrain_map，对地形点云的处理
    subTerrainCloud = nh.subscribe<sensor_msgs::PointCloud2>("/terrain_map", 5, &LpNode::terrainCloudHandler, this);
    // 回调获取目标点
    subGoal = nh.subscribe<geometry_msgs::PoseStamped>("/way_point", 5, &LpNode::goalHandler, this);
    // 控制joySpeed
    subSpeed = nh.subscribe<std_msgs::Float32>("/speed", 5, &LpNode::speedHandler, this);
    // 关闭局部地图标志位
    subCloseMap = nh.subscribe<std_msgs::Bool>("/close_map", 2, &LpNode::closeMapHandler, this);
    subGlobal_point = nh.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 2, &LpNode::globalPointHandler, this);

    // 发布局部规划路径
    pubPath = nh.advertise<nav_msgs::Path>("/local_path", 2);
    pubPannerAtuCloud = nh.advertise<sensor_msgs::PointCloud2>("/PannerAtuCloud", 5); // 实际参与规划的点云
    pubMap = nh.advertise<sensor_msgs::PointCloud2>("/PannerMap", 5);                 // 实际参与规划的点云
    pubSlowDown = nh.advertise<std_msgs::Float32>("/slow_down", 1);
    pubVirHeadDir = nh.advertise<geometry_msgs::PoseStamped>("/virture_head", 5);
    pubGlobalPath = nh.advertise<nav_msgs::Path>("/path_global", 2);
    pubAddPoints = nh.advertise<sensor_msgs::PointCloud2>("/add_points", 5);

// 开启可行路径可视化，发布可行路径
#if PLOTPATHSET == 1
    pubFreePaths = nh.advertise<sensor_msgs::PointCloud2>("/free_paths", 2);
#endif
    // 计算速度控制的K值
    planRangeK = pow(4, maxSpeed) / (lctlPtr->get_params().adjacentRange - lctlPtr->get_params().minPathRange);
    ROS_WARN("planRangeK: %f", planRangeK);
    makerInit();
}

/*
*********************************************************************************************************************************************************************
*********************************************************************************************************************************************************************
*局部规划的主算法运行流程
    从读取预设的startPaths.ply、paths.ply、pathList.ply、correspondences.txt文件开始
*/
void LpNode::read_pathfile()
{
    printf("\nReading path files.\n");
    // 初始化点云栈
    for (int i = 0; i < laserCloudStackNum; i++)
    {
        laserCloudStack[i].reset(new pcl::PointCloud<pcl::PointXYZI>());
    }
    // 初始化刚启动时车体可走的路径(在rviz上显示为黄色的路径点集合)
    for (int i = 0; i < groupNum; i++)
    {
        startPaths[i].reset(new pcl::PointCloud<pcl::PointXYZ>());
    }
// 初始化可行路径的可视化数据容器
#if PLOTPATHSET == 1
    // todo paths和startPaths的区别是什么
    for (int i = 0; i < pathNum; i++)
    {
        paths[i].reset(new pcl::PointCloud<pcl::PointXYZI>());
    }
#endif
    // correspondences的意义是路径点和体素网格的索引
    for (int i = 0; i < gridVoxelNum; i++)
    {
        correspondences[i].resize(0);
    }
    // 设置体素栅格的大小为 或者 terrainVoxelSize (m)的三维正方体
    terrainDwzFilter.setLeafSize(terrainVoxelSize, terrainVoxelSize, terrainVoxelSize);
    // 读取startPaths.ply文件，并将点云信息传入到startPaths中
    readStartPaths();
#if PLOTPATHSET == 1
    // 读取paths.ply文件，把点云信息存入到paths中
    readPaths();
#endif
    // 读取pathList.ply文件,目的是pathList将路径和对应的组id相对应，同时获取各个路径的目标点方向endDirPathList
    // todo readPathList遍历了343个path值，正常不是36个方向，36*pathnum343个路径吗
    // 全部方向的路径都是一样的，只是指定的方向不同(这是初始的方向，他会根据位移不断的更新)
    readPathList();
    // 读取correspondences.txt文件,目的是将体素网格和路径相对应
    readCorrespondences();
    printf("\real_files complete.\n\n");
}

// 转车头系并按距离等裁剪点云，adjacentRange为规划考虑的点云的范围
void LpNode::transform_allCloud()
{
    transform_plannerCloud();
}

void LpNode::pathRange_from_speed()
{
    // 将车体的路径范围（黄色）限制在minPathRange以上
    // pathRange和pathScale的共同点都是跟车体行驶速度有关的量
    // pathScale还和进一步筛选点云有关，通过把点云缩减pathscale倍来调整碰撞检测的范围，速度越快，考虑的点云应该也多
    if (lctlPtr->get_params().pathRangeBySpeed)
    {
        pathRange = pow(4, joySpeed) / planRangeK + lctlPtr->get_params().minPathRange; // 指数曲线
        if (pathRange < lctlPtr->get_params().minSpeedRange)
            pathRange = lctlPtr->get_params().minSpeedRange;
    }
    if (pathRange < lctlPtr->get_params().minPathRange)
        pathRange = lctlPtr->get_params().minPathRange;

    pathScale = lctlPtr->get_params().defPathScale; // local_planner外的暂存值，很关键，他只会是预设定的maxPathScale，因为loacal_planner结束是都会让pathScale回到这个值
    // 如果开启了pathScaleBySpeed，那么path（黄色）随着速度的变化而变化,并且把pathScale的最小值限制在了minPathScale里
    if (lctlPtr->get_params().pathScaleBySpeed)
    {
        pathScale = lctlPtr->get_params().defPathScale * (joySpeed / maxSpeed);
    }
    if (pathScale < lctlPtr->get_params().minPathScale)
        pathScale = lctlPtr->get_params().minPathScale;
}
void LpNode::transform_goal()
{
    // 目标点转移到车体坐标系
    float relativeGoalX = ((goalX - vehicleX) * cosVehicleYaw + (goalY - vehicleY) * sinVehicleYaw);
    float relativeGoalY = (-(goalX - vehicleX) * sinVehicleYaw + (goalY - vehicleY) * cosVehicleYaw);
    // 计算车体到waypoint（目标点）之间的距离
    relativeGoalDis = sqrt(relativeGoalX * relativeGoalX + relativeGoalY * relativeGoalY);
    if (relativeGoalDis > 1.0)
    {
        actual_goalClearRange = lctlPtr->get_params().goalClearRange / 10;
    }
    else
    {
        actual_goalClearRange = lctlPtr->get_params().goalClearRange;
    }
    // 目标点转移到车体坐标系
    float relativeGoalX_global = ((goal_point.pose.position.x - vehicleX) * cosVehicleYaw + (goal_point.pose.position.y - vehicleY) * sinVehicleYaw);
    float relativeGoalY_global = (-(goal_point.pose.position.x - vehicleX) * sinVehicleYaw + (goal_point.pose.position.y - vehicleY) * cosVehicleYaw);
    // 计算车体到goalpoint（目标点）之间的距离
    relativeGoalDis_global = sqrt(relativeGoalX_global * relativeGoalX_global + relativeGoalY_global * relativeGoalY_global);

    // joyDir为目标点相对车体的yaw角
    joyDir = atan2(relativeGoalY, relativeGoalX) * 180 / PI;
}

void LpNode::local_planner()
{
    // 如果pathScale和pathRange都不是最小值
    while ((pathScale != lctlPtr->get_params().minPathScale && lctlPtr->get_params().usePathScale) || pathRange >= lctlPtr->get_params().minPathRange)
    {
        ROS_INFO("Planning...pathRange: %f", pathRange);
        // 向参数服务器设置参数pathRange
        nhPrivate.setParam("pathRange", pathRange);
        // 动态调整规划方向,更小的范围要先判断
        if (pathRange == lctlPtr->get_params().minPathRange)
        {
            lctlPtr->set_enlarge_dirThre(15);
        }
        else if (pathRange <= 1.2)
        {
            lctlPtr->set_enlarge_dirThre(5);
        }
        else
        {
            lctlPtr->set_enlarge_dirThre(0);
        }

        clear_Lists_score(); // 先清空数据
        plannerCloudCropSize = plannerCloudCrop->points.size();
        PannerAtuCloud->clear();    // 实际用于规划的点云
        int PannerAtuCloudsize = 0; // 实际用于规划的点云的数量
        minObsAngCW = -180.0;
        minObsAngCCW = 180.0;
        diameter = sqrt(lctlPtr->get_params().vehicleLength / 2.0 * lctlPtr->get_params().vehicleLength / 2.0 + lctlPtr->get_params().vehicleWidth / 2.0 * lctlPtr->get_params().vehicleWidth / 2.0);
        angOffset = atan2(lctlPtr->get_params().vehicleWidth, lctlPtr->get_params().vehicleLength) * 180.0 / PI;
        for (int i = 0; i < plannerCloudCropSize; i++)
        {
            float x = plannerCloudCrop->points[i].x * pathScale; // 先除了用来计算，后面发布规划路径点位和可行路径的时候会乘回来，使障碍物看得更远
            float y = plannerCloudCrop->points[i].y * pathScale;
            float h = plannerCloudCrop->points[i].intensity;
            // 计算车体到点云的距离
            float dis = sqrt(x * x + y * y);
            float cloudDir = atan2(y, x);
            while (cloudDir > M_PI || cloudDir < -M_PI)
            {
                if (cloudDir > M_PI)
                    cloudDir -= 2 * M_PI;
                else if (cloudDir < -M_PI)
                    cloudDir += 2 * M_PI;
            }
            cloudDir *= 180 / PI;
            // 用于规划的点云满足以下三个条件
            //  ①checkObstacle为true
            // *②只考虑规划路径范围内的点云，这是主要的，也可以说可以只看这个
            //  ③如果使用pathCropByGoal，超过目标点的点云就不用管了
            if (dis < pathRange && (dis <= (relativeGoalDis + actual_goalClearRange) || !lctlPtr->get_params().pathCropByGoal) && (dis <= (relativeGoalDis_global + lctlPtr->get_params().goalClearRange_global)) && lctlPtr->get_params().checkObstacle)
            {
                // 下文是对此段代码的解释
                // rotAng当前检索方向和车体的角度
                // joyDir是目标点相对车体的yaw角
                // 那么angDiff是目标方向相对当前待规划的路径方向的yaw角
                for (int rotDir = 0; rotDir < 36; rotDir++) // 10度一次循一周36个方向
                {
                    rotAng = (10.0 * rotDir - 180.0) * PI / 180;      // 控制-PI到PI
                    angDiff = fabs(joyDir - (10.0 * rotDir - 180.0)); // 当前待规划的路径方向和目标方向的差值绝对值
                    // 将angDiff的范围映射到[0,180]
                    if (angDiff > 180.0)
                    {
                        angDiff = 360.0 - angDiff;
                    }
                    // 下列这一段代码的目的就是限定车体的搜索范围
                    if (angDiff > lctlPtr->get_params().dirThre) // 在搜索范围外，退出
                    {
                        continue;
                    }
                    PannerAtuCloud->push_back(plannerCloudCrop->points[i]); // 用于规划的点云
                    PannerAtuCloudsize++;
                    // 待规划点云转到当前规划组别方向的参考系
                    float x2 = cos(rotAng) * x + sin(rotAng) * y;
                    float y2 = -sin(rotAng) * x + cos(rotAng) * y;

                    grid_Synchronize_obstacles_to_paths(rotDir, x2, y2, h); // 根据待规划的点云，根据点云高度遮挡减掉可行的路径
                }
            }
            // 满足以下所有条件限制转弯，一般都不开启
            // 1 点云到车体的距离（经过缩减）小于 车体的直径（经过缩减），
            // 2 点云相对车体正前方或侧方距离大于半车长度
            // 3 开启地形分析下点云到地面的距离大于obstacleHeightThre，或不开启地形分析
            // 4 打开侧向障碍物分析
            if (dis < diameter / pathScale && (fabs(x) > lctlPtr->get_params().vehicleRadius / pathScale || fabs(y) > lctlPtr->get_params().vehicleRadius / pathScale) && h > lctlPtr->get_params().obstacleHeightThre && checkRotObstacle)
            {
                local_diff_limitTurn(x, y); // 限制转弯角度
            }
        }
        // 发布实际用于规划的点云
        // x,y方向偏移量
        pcl::PointCloud<pcl::PointXYZI> PannerAtuCloud1;
        PannerAtuCloud1 = *PannerAtuCloud;
        for (int i = 0; i < PannerAtuCloudsize; i++)
        {
            PannerAtuCloud1.points[i].x = PannerAtuCloud->points[i].x * cosVehicleYaw - PannerAtuCloud->points[i].y * sinVehicleYaw + vehicleX;
            PannerAtuCloud1.points[i].y = PannerAtuCloud->points[i].x * sinVehicleYaw + PannerAtuCloud->points[i].y * cosVehicleYaw + vehicleY;
            PannerAtuCloud1.points[i].z += vehicleZ;
        }
        sensor_msgs::PointCloud2 PannerAtuCloud2;
        pcl::toROSMsg(PannerAtuCloud1, PannerAtuCloud2);
        PannerAtuCloud2.header.stamp = ros::Time().fromSec(odomTime);
        PannerAtuCloud2.header.frame_id = "map";
        pubPannerAtuCloud.publish(PannerAtuCloud2);

        // 发布PannerAtuCloud中最近的障碍物的距离
        float min_distance = 6666; // 初始化为一个大值
        for (const auto &point : PannerAtuCloud->points)
        {
            float distance = sqrt(point.x * point.x + point.y * point.y);
            if (distance < min_distance && point.intensity > lctlPtr->get_params().obstacleHeightThre)
            {
                min_distance = distance;
            }
        }
        std_msgs::Float32 min_distance_msg;
        min_distance_msg.data = min_distance;
        pubSlowDown.publish(min_distance_msg);

        // 正式开始路径规划寻找最大分数的路径组
        for (int i = 0; i < 36 * pathNum; i++)
        {
            // 先计算出方向角
            int rotDir = int(i / pathNum);
            angDiff = fabs(joyDir - (10.0 * rotDir - 180.0));
            if (angDiff > 180.0)
            {
                angDiff = 360.0 - angDiff;
            }

            if (angDiff > lctlPtr->get_params().dirThre)
            {
                continue;
            }
            count_PathPerGroupScore(i, rotDir); // 以路径组为单位算得分
        }
        float maxScore = 0;
        int selectedGroupID = -1;                   // 得分最高路径组的ID，一个方向343条路径，分为7个组
        select_from_dir(maxScore, selectedGroupID); // 引用传值，根据方向进一步率选最大得分的路径组
        // 在选择出一组最优路径组后，根据startPaths组成一条最终的局部路径path，startPaths在文件中读取
        if (selectedGroupID >= 0)
        {
            pubPath_fromStartPaths(selectedGroupID);
#if PLOTPATHSET == 1
            pub_allFreePath(); // 以车头方向为参考系，发布所有的无障碍物路径
#endif
            pathFound = true;
            // 确认找到路之后，如果在最小范围内找到了最优路径，就把add_point_radius设置为0.75倍
            if (pathRange <= lctlPtr->get_params().minPathRange + 3 * lctlPtr->get_params().pathRangeStep)
            {
                lctlPtr->set_add_point_radius(0.75);
                ROS_INFO("set_add_point_radius rate: 0.8 .");
            }
            else if (pathRange <= lctlPtr->get_params().minPathRange + lctlPtr->get_params().pathRangeStep)
            {
                lctlPtr->set_add_point_radius(0.5);
                ROS_INFO("set_add_point_radius rate: 0.5 .");
            }
            else
            {
                lctlPtr->set_add_point_radius(1); // 重置为默认
                ROS_INFO("set_add_point_radius: default .");
            }
            // 找到了路，下次就可以扩大搜索范围
            if (pathRange < lctlPtr->get_params().adjacentRange - lctlPtr->get_params().pathRangeStep)
            {
                pathRange += lctlPtr->get_params().pathRangeStep;
            }
            else
            {
                pathRange = lctlPtr->get_params().adjacentRange;
            }
            break;
        }
        else // 找不到最优路径，缩短路径规模(减小碰撞范围)再找
        {
            pub_allFreePath();
            // 先让Range最小，一点点减少scale，膨胀周围点云
            // 注意，这里浮点数比较，一个float，一个double，精度不同导致一直不相等
            if (fabs(pathRange - lctlPtr->get_params().minPathRange) < 0.01)
            {
                // 如果在最小范围内还找不到路，直接把add_point_radius设置为0.8倍
                lctlPtr->set_add_point_radius(0.5);
                break;
            }
            // 先把规模调成0
            if (pathScale >= lctlPtr->get_params().minPathScale + lctlPtr->get_params().pathScaleStep && lctlPtr->get_params().usePathScale)
            {
                pathScale -= lctlPtr->get_params().pathScaleStep;
            }
            else if (pathScale <= lctlPtr->get_params().minPathScale - lctlPtr->get_params().pathScaleStep && lctlPtr->get_params().usePathScale)
            {
                pathScale += lctlPtr->get_params().pathScaleStep;
            }
            else
            {
                pathScale = lctlPtr->get_params().minPathScale;
                if (pathRange > lctlPtr->get_params().minPathRange)
                {
                    if (pathRange > 2.0) // 开始的规划范围快速衰减
                    {
                        pathRange -= 2 * lctlPtr->get_params().pathRangeStep;
                    }
                    else
                    {
                        pathRange -= lctlPtr->get_params().pathRangeStep;
                    }
                    if (pathRange < lctlPtr->get_params().minPathRange)
                        pathRange = lctlPtr->get_params().minPathRange;
                }
            }
        }
    }
}

void LpNode::fail_local_planner()
{
    ROS_INFO("Path not found!");
    nav_msgs::Path path;
    path.header.stamp = ros::Time().now();
    path.header.frame_id = robotFrame;
    path.poses.resize(1); // 路径规划失败，只发布一个原地点
    path.poses[0].pose.position.x = 0;
    path.poses[0].pose.position.y = 0;
    path.poses[0].pose.position.z = 0;
    path.poses[0].pose.orientation.w = 1;
    path.poses[0].header.frame_id = robotFrame;
    path.poses[0].header.stamp = ros::Time().now();
    pubPath.publish(path);

    // 转成世界坐标系后重新发布，方便可视化
    path.header.frame_id = "map";
    path.poses[0].pose.position.x = vehicleX;
    path.poses[0].pose.position.y = vehicleY;
    path.poses[0].pose.position.z = vehicleZ;
    path.poses[0].pose.orientation.w = 1;
    path.poses[0].header.frame_id = "map";
    path.poses[0].header.stamp = ros::Time().now();
    pubGlobalPath.publish(path);

#if PLOTPATHSET == 1
    freePaths->clear();
    sensor_msgs::PointCloud2 freePaths2;
    pcl::toROSMsg(*freePaths, freePaths2);
    freePaths2.header.stamp = ros::Time().now();
    freePaths2.header.frame_id = robotFrame;
    pubFreePaths.publish(freePaths2);
#endif
}

/*
*********************************************************************************************************************************************************************
*********************************************************************************************************************************************************************

    以下是在主算法中不直接调用，而是被其他函数间接调用的函数
*/
void LpNode::transform_plannerCloud()
{
    sinVehicleYaw = sin(vehicleYaw);
    cosVehicleYaw = cos(vehicleYaw);
    plannerCloudCrop->clear();
    // 遍历plannerCloud
    for (long unsigned int i = 0; i < plannerCloud->points.size(); i++)
    {
        // 减去车体的位置，目的就是从map坐标系回到车体坐标系
        float pointX1 = plannerCloud->points[i].x - vehicleX;
        float pointY1 = plannerCloud->points[i].y - vehicleY;
        float pointZ1 = plannerCloud->points[i].z - vehicleZ;
        // 将世界坐标系中的距离信息，转换成车体坐标系下的距离信息（以车头为x方向重新建系，算点云的坐标）
        /*
                   y   x车
            y 车   |  /
                 \ | /'yaw
                  \|/__ __ __ x
        */
        point.x = pointX1 * cosVehicleYaw + pointY1 * sinVehicleYaw;
        point.y = -pointX1 * sinVehicleYaw + pointY1 * cosVehicleYaw;
        point.z = pointZ1;
        point.intensity = plannerCloud->points[i].intensity;
        plannerCloudCrop->push_back(point);
    }
    // 遍历addedObstaclesCrop
    if (get_addedObstacles)
    {
        for (long unsigned int i = 0; i < addedObstaclesCrop->points.size(); i++)
        {
            float pointX1 = addedObstaclesCrop->points[i].x - vehicleX;
            float pointY1 = addedObstaclesCrop->points[i].y - vehicleY;
            float pointZ1 = addedObstaclesCrop->points[i].z - vehicleZ;
            point.x = pointX1 * cosVehicleYaw + pointY1 * sinVehicleYaw;
            point.y = -pointX1 * sinVehicleYaw + pointY1 * cosVehicleYaw;
            point.z = pointZ1;
            point.intensity = addedObstaclesCrop->points[i].intensity;
            plannerCloudCrop->push_back(point);
        }
        get_addedObstacles = false;
    }
}

void LpNode::clear_Lists_score()
{
    // 36 * pathNum的意义是36个方向的所有路径点集合，也就是车体360°的所有路径点集合
    // clearPathList代表了会被清除掉的路径信息
    // pathPenaltyList代表了路径的惩罚项
    for (int i = 0; i < 36 * pathNum; i++)
    {
        clearPathList[i] = 0;
        pathPenaltyList[i] = 0;
    }
    // 36 * groupNum的意义是36个方向所有的路径组，也就是车体360°的所有路径组
    // clearPathPerGroupScore代表了每一组路径的得分
    for (int i = 0; i < 36 * groupNum; i++)
    {
        clearPathPerGroupScore[i] = 0;
    }
}

void LpNode::grid_Synchronize_obstacles_to_paths(int rotDir, float x2, float y2, float h)
{
    // 下面这一段代码的意义是根据车体可能的转向，将激光点转换到相应的坐标系下，也就是转换到每一条路径下
    // 因为在生成路径和体素网格时，已经保存了体素网格和路径之间的索引关系，那么只需要计算网格的ID号，便可以知道哪些路径会被遮挡
    float scaleY = x2 / gridVoxelOffsetX + searchRadius / gridVoxelOffsetY * (gridVoxelOffsetX - x2) / gridVoxelOffsetX;
    // 计算体素网格的索引
    int indY = int((gridVoxelOffsetY + gridVoxelSize / 2 - y2 / scaleY) / gridVoxelSize);
    int indX = int((gridVoxelOffsetX + gridVoxelSize / 2 - x2) / gridVoxelSize);
    // 确保不会越界z
    if (indX >= 0 && indX < gridVoxelNumX && indY >= 0 && indY < gridVoxelNumY) // 找到障碍物（点云）对应网格
    {
        int ind = gridVoxelNumY * indX + indY;
        // 当前序号的体素网格,占据了多少条路径
        int blockedPathByVoxelNum = correspondences[ind].size();
        for (int j = 0; j < blockedPathByVoxelNum; j++) // 查看网络上的规划路径，j代表一条路径
        {
            // 使用地面分割的情况下当前激光点的高度大于obstacleHeightThre阈值,或者未使用地面分割时,则累加
            // 当一条路径上存在两个障碍点，即 pointPerPathThre=2，该路径才会认为被遮挡，所以只需要对未被遮挡的路径进行筛选
            if (h > lctlPtr->get_params().obstacleHeightThre) // 这一个点云障碍物将影响整个网格上的路径
            {
                // correspondences的空间161 * 451
                clearPathList[pathNum * rotDir + correspondences[ind][j]]++; // 将这个网格下的所有路径都加一个障碍物点
            }
            else
            {
                // 在使用了地面分割且激光点分割后高度小于障碍物高度阈值obstacleHeightThre时
                // 并且 当前高度大于原有路径惩罚值且大于地面高度阈值groundHeightThre
                // 障碍物越高惩罚值越高
                if (pathPenaltyList[pathNum * rotDir + correspondences[ind][j]] < h && h > lctlPtr->get_params().groundHeightThre)
                {
                    pathPenaltyList[pathNum * rotDir + correspondences[ind][j]] = h; // 如果这个障碍物点比前一个障碍物点还高，取最高
                }
            }
        }
    }
}

void LpNode::count_PathPerGroupScore(int pathId, int rotDir)
{
    if (clearPathList[pathId] <= lctlPtr->get_params().pointPerPathThre) // 检测到低于多个不可避免障碍物点云才进来规划，多个应该是防止噪声点云，不做地形分析下if必不满足
    {
        // 计算惩罚项的得分，惩罚项是根据地面阈值和障碍物阈值之间的点云高度计算得到的
        // 所以pathPenaltyList越高，惩罚得分penaltyScore也就越低
        // useCost置false后pathPenaltyList全是0
        float penaltyScore = 1.0 - pathPenaltyList[pathId] / lctlPtr->get_params().costHeightThre; // 减去高度，高度越高得分越低
        if (penaltyScore < lctlPtr->get_params().costScore)
            penaltyScore = lctlPtr->get_params().costScore;

        // 目标点和当前规划路径的夹角
        float dirDiff = fabs(joyDir - endDirPathList[pathId % pathNum] - (10.0 * rotDir - 180.0));
        // 这里将dirDiff的范围映射到[-180,180]
        if (dirDiff > 360.0)
        {
            dirDiff -= 360.0;
        }
        if (dirDiff > 180.0)
        {
            dirDiff = 360.0 - dirDiff;
        }

        // 越相对左右方向得分越小
        float rotDirW;
        if (rotDir < 18)
            rotDirW = fabs(fabs(rotDir - 9) + 1);
        else
            rotDirW = fabs(fabs(rotDir - 27) + 1);
        float score = 0;
        if (relativeGoalDis > 1.2)
        {
            // 计算惩罚得分，dirDiff目标点和当前路径夹角越小越高分，rotDirW初始方向不在正左右方越高，penaltyScore点云高度越低越高
            score = (1 - sqrt(sqrt((1 / lctlPtr->get_params().dirWeight) * dirDiff))) * abs(rotDirW * rotDirW * rotDirW) * penaltyScore;
        }
        else
        {
            score = pow((1 - sqrt(sqrt((1 / lctlPtr->get_params().dirWeight) * dirDiff))), 4) * abs(rotDirW * rotDirW * rotDirW) * penaltyScore;
        }
        // float score = (1 - sqrt(sqrt((1 / dirWeight) * dirDiff))) * penaltyScore;
        // 下面代码的作用简述是 ：
        // clearPathPerGroupScore 是一个 7 × 36 7×36 7×36的数组，即7组路径，36个方向，代表了7组在36个方向下的总得分
        // 接下来遍历这 7 × 36 = 252 7×36 = 252 7×36=252条路径，选取一条得分maxScore最高的路径selectedGroupID
        if (score > 0)
        {
            clearPathPerGroupScore[groupNum * rotDir + pathList[pathId % pathNum]] += score; // 计算路径组总得分
        }
    }
}

void LpNode::makerInit()
{
    // 设置可视化的形状
    shape = visualization_msgs::Marker::CUBE;
    marker.header.frame_id = "map";
    marker.ns = "LpNode"; // 拥有相同命名空间和id的marker会被上一个覆盖掉
    marker.id = 0;
    marker.type = shape;
    // 可以设置为 ADD, DELETE, 和 new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;
    // 设置marker相对于frame_id的位姿
    marker.pose.position.x = goalClearCenter_x;
    marker.pose.position.y = goalClearCenter_y;
    marker.pose.position.z = 0;

    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    // 立方体大小
    marker.scale.x = 0;
    marker.scale.y = 0;
    marker.scale.z = 0.5;
    // 颜色和透明度
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 0.3;
    marker.lifetime = ros::Duration(); // 括号内不设置时间为0时长，会在下一个maker到来时马上删除上一个
}

void LpNode::local_diff_limitTurn(int x, int y)
{
    // 除此之外还存在一种情况，这里应该是对于差速底盘的情况
    // 障碍物不在前面而是在侧面，此时便不能通过路径点集合去寻找，因为在转向的过程中可能会碰撞
    // 所以如果有这种点云的出现，那么需要对此时的局部路径做出一些限制
    // 首先判断是否存在这种点云，即点云距离小于 diameter 车辆半径 ，但点云不在车体内部， 并且超过了障碍物阈值
    // 那么需要根据点云在左侧还是右侧，对此刻的转向进行限制
    // 其中CW即顺时针旋转（Clock Wise）的方向 与CW反方向旋转时为CCW （Counter Clock Wise）
    float angObs = atan2(y, x) * 180.0 / PI;

    // 障碍点云在车体的左侧
    if (angObs > 0)
    {
        // 考虑车体转弯后，限制了车体需要转弯的角度[-180,180]
        if (minObsAngCCW > angObs - angOffset)
            minObsAngCCW = angObs - angOffset;
        if (minObsAngCW < angObs + angOffset - 180.0)
            minObsAngCW = angObs + angOffset - 180.0;
    }
    // 障碍点云在车体的右侧
    else
    {
        if (minObsAngCW < angObs + angOffset)
            minObsAngCW = angObs + angOffset;
        if (minObsAngCCW > 180.0 + angObs - angOffset)
            minObsAngCCW = 180.0 + angObs - angOffset;
    }
    if (minObsAngCW > 0)
        minObsAngCW = 0;
    if (minObsAngCCW < 0)
        minObsAngCCW = 0;
}

void LpNode::select_from_dir(float &maxScore, int &selectedGroupID)
{
    for (int i = 0; i < 36 * groupNum; i++)
    {
        int rotDir = int(i / pathNum);
        float rotAng = (10.0 * rotDir - 180.0) * PI / 180;
        float rotDeg = 10.0 * rotDir; // 和上面唯一的区别就是这个没有修正到-180到180
        if (rotDeg > 180.0)
            rotDeg -= 360.0;
        // 这里在对侧方碰撞进行限制了之后，算出了最高得分的路径组的ID
        if (maxScore < clearPathPerGroupScore[i] && ((rotAng * 180.0 / PI > minObsAngCW && rotAng * 180.0 / PI < minObsAngCCW) // 限制转弯
                                                     || !checkRotObstacle))
        {
            maxScore = clearPathPerGroupScore[i];
            selectedGroupID = i; // 选出来的路径时路径组中的某一条，还不是路径组
        }
    }
}

void LpNode::close_map()
{
    if (lctlPtr->get_params().use_map)
    {
        ROS_INFO("Close local map!");
        lctlPtr->set_use_map(false);
        close_map_begin = ros::Time::now().toSec();
    }
}

void LpNode::load_pcd_map()
{
    // 加载允许下台阶的点云图
    pcl::io::loadPCDFile(usual_pcd_path, terrainMapRecord);
    terrainMapRecord.header.frame_id = "map";
    terrainMapRecord.header.stamp = ros::Time::now();
    pcl::fromROSMsg(terrainMapRecord, *terrainMapRecord_pcl);
    return;
}