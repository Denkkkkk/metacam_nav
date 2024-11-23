#include "relocalization.h"
#include "std_msgs/Bool.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

std_msgs::Bool need_reloc;
int count;

Scan2MapLocation::Scan2MapLocation() : scan_resoult(new sensor_msgs::PointCloud2), private_node_("~"), tf_listener_(tfBuffer_)
{
    // \033[1;32m，\033[0m 终端显示成绿色
    ROS_INFO_STREAM("\033[1;32m----> ICP location started.\033[0m");

    // 初始化订阅者
    laser_scan_subscriber_ = node_handle_.subscribe("/terrain_map", 1, &Scan2MapLocation::ScanCallback,
                                                    this, ros::TransportHints().tcpNoDelay());
    odom_subscriber_ = node_handle_.subscribe("/Odometry", 20, &Scan2MapLocation::OdomCallback,
                                              this, ros::TransportHints().tcpNoDelay());

    scan_pointcloud_publisher_ = node_handle_.advertise<sensor_msgs::PointCloud2>("scan_pointcloud", 10);

    scan_map_publisher_ = node_handle_.advertise<sensor_msgs::PointCloud2>("scan_map", 10);

    removal_pointcloud_publisher_ = node_handle_.advertise<sensor_msgs::PointCloud2>("removal_pointcloud", 10);

    // 注意，这里的发布器，发布的数据类型为 PointCloudT
    // ros中自动做了 PointCloudT 到 sensor_msgs/PointCloud2 的数据类型的转换
    icp_pointcloud_publisher_ = node_handle_.advertise<PointCloudT>("icp_pointcloud", 1, this);

    rotate_pointcloud_publisher_ = node_handle_.advertise<PointCloudT>("rotate_pointcloud", 1, this);

    location_publisher_ = node_handle_.advertise<geometry_msgs::PoseWithCovarianceStamped>("location_match", 1, this);

    relocate_tranform_visuial_publisher_ = node_handle_.advertise<geometry_msgs::PoseWithCovarianceStamped>("relocate_visuial_pose", 1, this);

    match_point_publisher_ = node_handle_.advertise<geometry_msgs::PoseStamped>("relocalization", 1, this);

    need_relocal_sub = node_handle_.subscribe<std_msgs::Bool>("/need_reloc", 1, &Scan2MapLocation::needRelocCallBack, this);

    vehicle_publisher_ = node_handle_.advertise<sensor_msgs::PointCloud2>("rector_points", 1, this);

    safetyStop_publisher_ = node_handle_.advertise<std_msgs::Bool>("/stop", 5, this);
    // relocalization_srv_ = node_handle_.advertiseService("relocat_srv", &Scan2MapLocation::RelocalizeCallback, this);

    // 参数初始化
    InitParams();
    cloud_map_ = boost::shared_ptr<PointCloudT>(new PointCloudT());
    // 指针的初始化
    sensor_msgs::PointCloud2 pc2_temp;
    if (pcd_path != "-1")
    {
        pcl::io::loadPCDFile(pcd_path, pc2_temp);
        pc2_temp.header.frame_id = "map";
        pc2_temp.header.stamp = ros::Time::now();
        pcl::fromROSMsg(pc2_temp, *cloud_map_);
        map_initialized_ = true;
    }
}

/*
 * 的参数初始化
 */
void Scan2MapLocation::InitParams()
{
    private_node_.param<bool>("save_pcd", save_pcd_, true);

    private_node_.param<std::string>("odom_frame", odom_frame_, "Sentry/odom");
    private_node_.param<std::string>("base_frame", base_frame_, "Sentry/vehicle");
    private_node_.param<std::string>("map_frame", map_frame_, "map");
    private_node_.param<std::string>("lidar_frame", lidar_frame_, "mid360");

    private_node_.param<int>("odom_queue_length", odom_queue_length_, 300);

    // 发布位姿的方差
    private_node_.param<double>("Variance_X", Variance_X, 0.01);     // x方向上方差
    private_node_.param<double>("Variance_Y", Variance_Y, 0.01);     // y方向上方差
    private_node_.param<double>("Variance_Yaw", Variance_Yaw, 0.01); // yaw方向上方差

    // 定位禁区
    private_node_.param<std::vector<double>>("location_restricted_zone", location_restricted_zone_, {});

    // 重定位
    private_node_.param<double>("Relocation_Weight_Score", Relocation_Weight_Score_, 0.5);            // 重定位分数系数
    private_node_.param<double>("Relocation_Weight_Distance", Relocation_Weight_Distance_, 0.5);      // 重定位距离系数
    private_node_.param<double>("Relocation_Weight_Yaw", Relocation_Weight_Yaw_, 0.5);                // 重定位yaw角系数
    private_node_.param<double>("Relocation_Maximum_Iterations", Relocation_Maximum_Iterations_, 60); // 重定位最大迭代次数
    private_node_.param<double>("Relocation_Score_Threshold_Max", Relocation_Score_Threshold_Max_, 0.15);
    private_node_.param<double>("obstacleHeightThre", obstacleHeightThre, 0.08);
    private_node_.param<std::string>("pcd_path", pcd_path, "-1");

    // 扇形点云
    private_node_.param<double>("sector_angle1", sector_angel1, 0);
    private_node_.param<double>("sector_angle2", sector_angel2, 180);
    private_node_.param<double>("sector_angle3", sector_angel3, -0);
    private_node_.param<double>("sector_angle4", sector_angel4, -180);
}

void Scan2MapLocation::needRelocCallBack(const std_msgs::Bool::ConstPtr &need_status)
{
    need_reloc.data = need_status->data;
}

/*
 * Odom回调函数
 */
void Scan2MapLocation::OdomCallback(const nav_msgs::Odometry::ConstPtr &odometry_msg)
{
    std::lock_guard<std::mutex> lock(odom_lock_);
    odom_initialized_ = true;

    double roll, pitch, yaw;
    geometry_msgs::Quaternion geoQuat = odometry_msg->pose.pose.orientation;
    tf::Matrix3x3(tf::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w)).getRPY(roll, pitch, yaw);
    vehicleX = odometry_msg->pose.pose.position.x;
    vehicleY = odometry_msg->pose.pose.position.y;
    vehicleZ = odometry_msg->pose.pose.position.z;
    vehicleYaw = yaw;
}

/*
 * Scan回调函数 进行数据处理
 */
void Scan2MapLocation::ScanCallback(const sensor_msgs::PointCloud2::ConstPtr &scan_msg)
{
    // 发布地图点云
    sensor_msgs::PointCloud2 map_pointcloud;
    pcl::toROSMsg(*cloud_map_, map_pointcloud);
    map_pointcloud.header.frame_id = "map";
    map_pointcloud.header.stamp = ros::Time::now();
    scan_map_publisher_.publish(map_pointcloud);
    if (!need_reloc.data)
    {
        if (need_stop.data)
        {
            need_stop.data = false;
            safetyStop_publisher_.publish(need_stop);
        }
        return;
    }
    else
    {
        if (!need_stop.data)
        {
            need_stop.data = true;
            safetyStop_publisher_.publish(need_stop);
        }
    }
    // 输出提示
    ROS_INFO_STREAM("\033[1;32m----> ICP location started.\033[0m");
    scan_start_time_ = std::chrono::steady_clock::now(); // 保存时间，用于最后计时
    if (!GetTransform(map_to_base_, map_frame_, base_frame_, scan_msg->header.stamp))
    {
        ROS_WARN("Did not get base pose at now");
        return;
    }

    // 判断地图和里程计数据是否初始化，如果没有则退出
    if (!map_initialized_ || !odom_initialized_)
    {
        ROS_WARN("map or odom not initialized");
        return;
    }
    // 剔除地面点云
    pcl::PointCloud<pcl::PointXYZI> terrain_map;
    pcl::fromROSMsg(*scan_msg, terrain_map);
    pcl::PointXYZI scan_point;
    pcl::PointCloud<pcl::PointXYZI> scan_temp;
    for (int i = 0; i < terrain_map.points.size(); i++)
    {
        scan_point = terrain_map.points[i];
        if (scan_point.intensity > obstacleHeightThre)
        {
            scan_temp.push_back(scan_point);
        }
    }
    pcl::toROSMsg(scan_temp, *scan_resoult);
    scan_resoult->header.frame_id = scan_msg->header.frame_id;
    scan_resoult->header.stamp = scan_msg->header.stamp;
    scan_temp.clear();

    if (is_coordinate_in_range(location_restricted_zone_, map_to_base_))
    {
        ROS_INFO_STREAM("\033[1;33m-into restricted area, stop icp match.\033[0m");
        return;
    }

    // 选择扇形点云用来重定位
    pcl::PointCloud<pcl::PointXYZ> temp_map;
    pcl::PointCloud<pcl::PointXYZ> vehicle_points;
    double range1 = (sector_angel1 / 180) * M_PI;
    double range2 = (sector_angel2 / 180) * M_PI;
    double range3 = (sector_angel3 / 180) * M_PI;
    double range4 = (sector_angel4 / 180) * M_PI;
    // 转换到map坐标系
    fromROSMsg(*scan_resoult, temp_map);
    for (int i = 0; i < temp_map.points.size(); i++)
    {
        // 转到车体坐标系
        pcl::PointXYZ point;
        // 减去车体的位置，目的就是从map坐标系回到车体坐标系
        float pointX1 = temp_map.points[i].x - vehicleX;
        float pointY1 = temp_map.points[i].y - vehicleY;
        float pointZ1 = temp_map.points[i].z - vehicleZ;
        point.x = pointX1 * cos(vehicleYaw) + pointY1 * sin(vehicleYaw);
        point.y = -pointX1 * sin(vehicleYaw) + pointY1 * cos(vehicleYaw);
        point.z = pointZ1;
        double yaw = atan2(point.y, point.x);
        // 转到-PI to PI
        while (yaw > M_PI || yaw < -M_PI)
        {
            if (yaw > M_PI)
                yaw -= 2 * M_PI;
            else if (yaw < -M_PI)
                yaw += 2 * M_PI;
        }
        if ((yaw < range3 && yaw > range4) || (yaw > range1 && yaw < range2))
        {
            vehicle_points.push_back(point);
        }
    }
    // 发布vehice_points
    sensor_msgs::PointCloud2 vehicle_points_msg;
    pcl::toROSMsg(vehicle_points, vehicle_points_msg);
    vehicle_points_msg.header.stamp = ros::Time::now();
    vehicle_points_msg.header.frame_id = "map";
    vehicle_publisher_.publish(vehicle_points_msg);

    // 转到世界坐标系下
    pcl::PointCloud<pcl::PointXYZ> sector_points;
    for (int i = 0; i < vehicle_points.points.size(); i++)
    {
        pcl::PointXYZ point;
        point.x = vehicle_points[i].x * cos(vehicleYaw) - vehicle_points[i].y * sin(vehicleYaw) + vehicleX;
        point.y = vehicle_points[i].x * sin(vehicleYaw) + vehicle_points[i].y * cos(vehicleYaw) + vehicleY;
        point.z = vehicle_points[i].z + vehicleZ;
        sector_points.push_back(point);
    }
    pcl::toROSMsg(sector_points, *scan_resoult);
    scan_resoult->header.frame_id = "map";
    scan_resoult->header.stamp = ros::Time::now();

    // 重定位
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    Filter(scan_resoult, filtered_cloud);
    gicp_registration(match_result_, filtered_cloud, cloud_map_, map_to_base_);

    // 加上计算时位移
    // Eigen::Isometry3d basebegin_to_basenow = Eigen::Isometry3d::Identity();
    // if (!Get2TimeTransform(basebegin_to_basenow))
    // {
    //     ROS_WARN("Did not get base pose at now");
    //     return;
    // }

    // 将结果转为PoseStamped类型并发布
    // 旋转矩阵转四元数
    Eigen::Quaterniond q = Eigen::Quaterniond(match_result_.rotation());
    std_msgs::Header header;
    header.stamp = scan_resoult->header.stamp;
    header.frame_id = "map";
    location_match.header = header;
    location_match.pose.pose.orientation.x = q.x();
    location_match.pose.pose.orientation.y = q.y();
    location_match.pose.pose.orientation.z = q.z();
    location_match.pose.pose.orientation.w = q.w();
    // location_match.pose.orientation = q;
    location_match.pose.pose.position.x = match_result_.translation()(0);
    location_match.pose.pose.position.y = match_result_.translation()(1);
    location_match.pose.pose.position.z = match_result_.translation()(2);
    // x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis
    location_match.pose.covariance = {Variance_X, 0, 0, 0, 0, 0,
                                      0, Variance_Y, 0, 0, 0, 0,
                                      0, 0, 1e-9, 0, 0, 0,
                                      0, 0, 0, 1e-9, 0, 0,
                                      0, 0, 0, 0, 1e-9, 0,
                                      0, 0, 0, 0, 0, Variance_Yaw};
    location_publisher_.publish(location_match); //).sleep();
    if (best_score <= Relocation_Score_Threshold_Max_)
    {
        double roll_, pitch_, yaw_;
        tf::Matrix3x3(tf::Quaternion(q.x(), q.y(), q.z(), q.w())).getEulerYPR(yaw_, pitch_, roll_);
        tf::Quaternion quaternion;
        quaternion.setRPY(0, 0, yaw_);
        geometry_msgs::Quaternion pub_quaternion;
        tf::quaternionTFToMsg(quaternion, pub_quaternion);
        pub_match_result.pose.orientation.x = pub_quaternion.x;
        pub_match_result.pose.orientation.y = pub_quaternion.y;
        pub_match_result.pose.orientation.z = pub_quaternion.z;
        pub_match_result.pose.orientation.w = pub_quaternion.w;
        pub_match_result.pose.position.x = match_result_.translation()(0);
        pub_match_result.pose.position.y = match_result_.translation()(1);
        pub_match_result.pose.position.z = match_result_.translation()(2);
        match_point_publisher_.publish(pub_match_result);
        // ros::Duration(2.5).sleep();
        // if (count == 0)
        // {
        //     count++;
        // }
        // else if (count == 1)
        // {
        //     count = 0;
        //     need_reloc.data = false;
        // }
        need_reloc.data = false;
        // ROS_WARN("pushed match result:pub_quaternion.x=%f,pub_quaternion.y=%f,pub_quaternion.z=%f,pub_quaternion.w=%f,\nx=%f,y=%f,z=%f",
        //          pub_match_result.pose.orientation.x, pub_match_result.pose.orientation.y, pub_match_result.pose.orientation.z, pub_match_result.pose.orientation.w,
        //          pub_match_result.pose.position.x, pub_match_result.pose.position.y, pub_match_result.pose.position.z);
    }

    // 计算函数用时并发布
    scan_end_time_ = std::chrono::steady_clock::now();
    scan_time_used_ = std::chrono::duration_cast<std::chrono::duration<double>>(scan_end_time_ - scan_start_time_);

    std::cout << "ScanCallBack整体函数处理用时: " << scan_time_used_.count() << " 秒。" << std::endl;
    std::cout << "ScanCallback函数频率为: " << 1.0 / scan_time_used_.count() << " HZ。" << std::endl;
    scan_last_end_time_ = scan_end_time_;
}

/**
 * @brief 对点云数据绕机器人坐标点进行旋转
 *
 * @param cloud_msg 点云数据
 * @param rotation 旋转角度
 * @param robo_pose 机器人位姿
 */
void Scan2MapLocation::rotatePointCloud(PointCloudT::Ptr &cloud_msg, const Eigen::Affine3f &rotation, const Eigen::Affine3f &robo_pose)
{
    // 将点云转换到原点
    pcl::transformPointCloud(*cloud_msg, *cloud_msg, robo_pose.inverse());
    // 绕原点进行旋转
    pcl::transformPointCloud(*cloud_msg, *cloud_msg, rotation);
    // 将点云平移回去：
    pcl::transformPointCloud(*cloud_msg, *cloud_msg, robo_pose);
}

// 转换消息类型
geometry_msgs::PoseWithCovarianceStamped Scan2MapLocation::Isometry3d_to_PoseWithCovarianceStamped(const Eigen::Isometry3d &iso)
{
    // 创建一个PoseWithCovarianceStamped类型的消息
    geometry_msgs::PoseWithCovarianceStamped pose_msg;

    // 填充消息头
    pose_msg.header.stamp = ros::Time::now();
    pose_msg.header.frame_id = "map";

    // 旋转矩阵转四元数
    Eigen::Quaterniond q = Eigen::Quaterniond(iso.rotation());

    // 填充消息头
    std_msgs::Header header;
    header.stamp = ros::Time::now();
    header.frame_id = "map";
    pose_msg.header = header;
    pose_msg.pose.pose.orientation.x = q.x();
    pose_msg.pose.pose.orientation.y = q.y();
    pose_msg.pose.pose.orientation.z = q.z();
    pose_msg.pose.pose.orientation.w = q.w();
    // pose_msg.pose.orientation = q;
    pose_msg.pose.pose.position.x = iso.translation()(0);
    pose_msg.pose.pose.position.y = iso.translation()(1);
    pose_msg.pose.pose.position.z = iso.translation()(2);

    return pose_msg;
}

/**
 * 从tf树读取map到base_link两个时刻之间的坐标变换
 * 保存到trans中
 * trans 类型： Eigen::Isometry3d
 */
bool Scan2MapLocation::Get2TimeTransform(Eigen::Isometry3d &trans)
{
    Eigen::Isometry3d transBegin = Eigen::Isometry3d::Identity();
    transBegin = map_to_base_;

    Eigen::Isometry3d transEnd = Eigen::Isometry3d::Identity();
    // transEnd = Eigen::Isometry3d::Identity();     //map到base的欧式变换矩阵4x4
    geometry_msgs::TransformStamped transformStamped;

    try
    {
        transformStamped = tfBuffer_.lookupTransform(map_frame_, base_frame_, ros::Time(0), ros::Duration(0));
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("DIDNT GET TRANSFORM ON Get2TimeTransform");
        ros::Duration(1.0).sleep();
        return false;
    }
    tf2::Quaternion quat(
        transformStamped.transform.rotation.x,
        transformStamped.transform.rotation.y,
        transformStamped.transform.rotation.z,
        transformStamped.transform.rotation.w);

    double roll, pitch, yaw;                       // 定义存储r\p\y的容器
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw); // 进行转换
    // std::cout << "trans roll, pitch, yaw =  \n" << roll << "  " << pitch << "  " << yaw << std::endl;

    Eigen::Matrix3d point_rotation;
    point_rotation = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
                     Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                     Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());

    transEnd.rotate(point_rotation);
    transEnd.pretranslate(Eigen::Vector3d(transformStamped.transform.translation.x,
                                          transformStamped.transform.translation.y,
                                          transformStamped.transform.translation.z));

    // 计算得到这段时间的坐标变换
    trans = transEnd * transBegin.inverse();
    return true;
}

// 检查给定坐标是否在范围内
bool Scan2MapLocation::is_coordinate_in_range(const std::vector<double> &vec, const Eigen::Isometry3d &coord)
{
    if (vec.size() % 4 != 0)
    {
        ROS_INFO_STREAM("\033[1;31m----> size of vector is not a multiple of 4.\033[0m");
        return false;
    }

    const int num_ranges = vec.size() / 4;
    for (int i = 0; i < num_ranges; i++)
    {
        double x1 = vec[i * 4];
        double y1 = vec[i * 4 + 1];
        double x2 = vec[i * 4 + 2];
        double y2 = vec[i * 4 + 3];

        if (x1 <= coord.translation().x() && coord.translation().x() <= x2 &&
            y1 <= coord.translation().y() && coord.translation().y() <= y2)
        {
            return true;
        }
    }

    return false;
}

// 体素滤波和离群点滤波
void Scan2MapLocation::Filter(const sensor_msgs::PointCloud2::Ptr &scan_resoult, PointCloudT::Ptr filtered_cloud)
{
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*scan_resoult, *pcl_cloud);
    // 设置体素滤波器的叶子大小（体素边长）
    voxel_filter.setInputCloud(pcl_cloud);
    double leaf_size = 0.1;
    voxel_filter.setLeafSize(leaf_size, leaf_size, leaf_size);
    // 执行体素滤波，输出结果存储在滤波后的点云对象中
    voxel_filter.filter(*filtered_cloud);
    // 离群点滤波
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(filtered_cloud);
    sor.setMeanK(30);            // 平均值
    sor.setStddevMulThresh(0.6); // 标准差
    sor.filter(*filtered_cloud);
    pcl::toROSMsg(*filtered_cloud, *scan_resoult);
    scan_resoult->header.frame_id = "map";
    scan_resoult->header.stamp = ros::Time::now();
    scan_pointcloud_publisher_.publish(scan_resoult);
}

void Scan2MapLocation::gicp_registration(Eigen::Isometry3d &trans, PointCloudT::Ptr source, PointCloudT::Ptr target, const Eigen::Isometry3d &robot_pose)
{
    // // 从裁判系统读的机器人坐标及yaw
    // Eigen::Isometry3d globle_transform = Eigen::Isometry3d::Identity();
    // globle_transform.translation() << globle_x, globle_y, 0.0;
    // Eigen::AngleAxisd globle_rotation(globle_yaw, Eigen::Vector3d::UnitZ());
    // globle_transform.rotate(globle_rotation);

    // // 从tf传入的位姿读机器人坐标
    // Eigen::Vector3d robot_pose_ = robot_pose.translation();
    // double robot_pose_x = robot_pose_.x();
    // double robot_pose_y = robot_pose_.y();
    // Eigen::Quaterniond rotation_(robot_pose.rotation());
    // double robot_pose_yaw = rotation_.toRotationMatrix().eulerAngles(0, 1, 2)[2];

    // 设置不同的旋转角度来生成不同的初始变换矩阵
    std::vector<Eigen::Matrix4f> initial_transforms;
    std::vector<float> angles;
    std::vector<float> fitness_scores;
    std::vector<Eigen::Affine3f> Transformation_sources;
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> pointcloud_sources;
    pcl::PointCloud<pcl::PointXYZ>::Ptr icp_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    for (float angle = 0.0; angle < 360.0; angle += 20.0)
    {
        Eigen::Affine3f transform = Eigen::Affine3f::Identity();
        transform.rotate(Eigen::AngleAxisf(angle * M_PI / 180.0, Eigen::Vector3f::UnitZ()));

        initial_transforms.push_back(transform.matrix());
        angles.push_back(angle * M_PI / 180.0);
    }
    for (int i = 0; i < initial_transforms.size(); i++)
    {
        // 将 Matrix4f 类型数据转换为 Affine3f 类型
        Eigen::Affine3f initial_affine(initial_transforms[i]);
        PointCloudT::Ptr rotate_scan_cloud(new PointCloudT(*source));
        rotatePointCloud(rotate_scan_cloud, initial_affine, robot_pose.cast<float>());
        rotate_pointcloud_publisher_.publish(rotate_scan_cloud);

        //-----------------初始化GICP对象-------------------------
        pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;
        //-----------------KD树加速搜索---------------------------
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree1(new pcl::search::KdTree<pcl::PointXYZ>);
        tree1->setInputCloud(source);
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree2(new pcl::search::KdTree<pcl::PointXYZ>);
        tree2->setInputCloud(target);
        gicp.setSearchMethodSource(tree1);
        gicp.setSearchMethodTarget(tree2);
        //-----------------设置GICP相关参数-----------------------
        gicp.setInputSource(rotate_scan_cloud); // 源点云
        gicp.setInputTarget(target);            // 目标点云
        gicp.setMaxCorrespondenceDistance(1);   // 设置对应点对之间的最大距离(3)
        gicp.setTransformationEpsilon(1e-10);   // 为终止条件设置最小转换差异
        gicp.setEuclideanFitnessEpsilon(0.001); // 设置收敛条件是均方误差和小于阈值,停止迭代(0.0007)
        gicp.setMaximumIterations(80);          // 最大迭代次数(100)
        // gicp.setCorrespondenceRandomness(5);        // 设置随机性(5-10)
        // gicp.setUseReciprocalCorrespondences(true); // 使用相互对应关系
        // 计算需要的刚体变换以便将输入的源点云匹配到目标点云
        gicp.align(*icp_cloud);

        // 收敛了之后, 获取坐标变换
        Eigen::Affine3f transfrom = Eigen::Affine3f::Identity();
        transfrom = gicp.getFinalTransformation();

        // 将Eigen::Affine3f转换成x, y, theta
        float x, y, z, roll, pitch, yaw;
        pcl::getTranslationAndEulerAngles(transfrom, x, y, z, roll, pitch, yaw);

        double tranDist = sqrt(x * x + y * y);
        double angleDist = abs(yaw);

        // float fitness_score = Relocation_Weight_Score_ * gicp.getFitnessScore() +
        //                       Relocation_Weight_Distance_ * tranDist + Relocation_Weight_Yaw_ * angleDist;
        float fitness_score = gicp.getFitnessScore();
        fitness_scores.push_back(fitness_score);
        Transformation_sources.push_back(transfrom);
        icp_cloud->header.frame_id = "map";
        pointcloud_sources.push_back(icp_cloud);
        icp_pointcloud_publisher_.publish(icp_cloud); // 发布匹配结果

        // --------------------可视化每次匹配得到的坐标数据----------------

        // 将initial_transform从Matrix4f转换为Isometry3d
        Eigen::Isometry3d initial_transform = Eigen::Isometry3d::Identity();
        initial_transform.linear() = initial_transforms[i].cast<double>().matrix().block<3, 3>(0, 0);
        initial_transform.translation() = initial_transforms[i].cast<double>().matrix().block<3, 1>(0, 3);

        Eigen::Isometry3d transfrom_icp = Eigen::Isometry3d::Identity();
        transfrom_icp.matrix() = transfrom.matrix().cast<double>();

        transfrom_icp = robot_pose.inverse() * transfrom_icp * robot_pose;

        // 将icp结果从Matrix4f类型转换为Isometry3d类型，并点乘初始变换
        Eigen::Isometry3d relocate_tranform_visuial_ = Eigen::Isometry3d::Identity(); // 重定位过程可视化坐标
        relocate_tranform_visuial_ = robot_pose * transfrom_icp * initial_transform;

        geometry_msgs::PoseWithCovarianceStamped relocation_match_visual; // 重定位可视化结果
        relocation_match_visual = Isometry3d_to_PoseWithCovarianceStamped(relocate_tranform_visuial_);
        relocate_tranform_visuial_publisher_.publish(relocation_match_visual);
    }
    ROS_INFO_STREAM("\033[1;33m icp finished.\033[0m");

    // 找到分数最高的匹配结果
    int best_index = 0;
    best_score = fitness_scores[0];
    for (int i = 1; i < fitness_scores.size(); i++)
    {
        if (fitness_scores[i] < best_score)
        {
            best_index = i;
            best_score = fitness_scores[i];
        }
    }
    // 将initial_transform从Matrix4f转换为Isometry3d
    Eigen::Isometry3d initial_transform = Eigen::Isometry3d::Identity();
    initial_transform.linear() = initial_transforms[best_index].cast<double>().matrix().block<3, 3>(0, 0);
    initial_transform.translation() = initial_transforms[best_index].cast<double>().matrix().block<3, 1>(0, 3);

    // 将icp结果从Matrix4f转换为Isometry3d，并转换到robot_pose
    Eigen::Isometry3d transfrom_icp = Eigen::Isometry3d::Identity();
    transfrom_icp.matrix() = Transformation_sources[best_index].matrix().cast<double>();
    transfrom_icp = robot_pose.inverse() * transfrom_icp * robot_pose;

    // 将icp结果从Matrix4f类型转换为Isometry3d类型，并点乘初始变换
    trans = robot_pose * transfrom_icp * initial_transform;
    double x = trans.translation().x();
    double y = trans.translation().y();
    Eigen::Matrix3d rotationMatrix = trans.rotation();
    double yaw = atan2(rotationMatrix(1, 0), rotationMatrix(0, 0)); // 使用反正切函数计算yaw角度
    // 输出匹配结果
    std::cout << "---------------------------------" << std::endl;
    std::cout << "x:" << x << " y: " << y << " yaw: " << yaw << std::endl;
    std::cout << "Best fitness score:" << std::endl
              << best_score << std::endl;

    if (best_score > Relocation_Score_Threshold_Max_)
    {
        ROS_INFO_STREAM("\033[1;33m faile to relocalize.\033[0m");
    }

    // 计算对齐点云的均值标准差
    pcl::registration::CorrespondenceEstimation<pcl::PointXYZ, pcl::PointXYZ> core;
    core.setInputSource(icp_cloud);
    core.setInputTarget(target);
    boost::shared_ptr<pcl::Correspondences> cor(new pcl::Correspondences);
    core.determineCorrespondences(*cor, 5);
    double mean = 0, stddev = 0;
    pcl::registration::getCorDistMeanStd(*cor, mean, stddev);
    std::cout << mean << "---------" << stddev << endl;
}

bool Scan2MapLocation::GetTransform(Eigen::Isometry3d &trans, const std::string parent_frame, const std::string child_frame, const ros::Time stamp)
{
    bool gotTransform = false;
    trans = Eigen::Isometry3d::Identity(); // map到base的欧式变换矩阵4x4
    geometry_msgs::TransformStamped transformStamped;

    try
    {
        gotTransform = true;
        transformStamped = tfBuffer_.lookupTransform(parent_frame, child_frame, stamp, ros::Duration(1.0));
        // std::cout << "input rostime of Transform:" << stamp <<std::endl;
    }
    catch (tf2::TransformException &ex)
    {
        gotTransform = false;
        ROS_WARN("DIDNT GET TRANSFORM %s %s IN B", parent_frame.c_str(), child_frame.c_str());
        ros::Duration(1.0).sleep();
        return false;
    }

    tf2::Quaternion quat(
        transformStamped.transform.rotation.x,
        transformStamped.transform.rotation.y,
        transformStamped.transform.rotation.z,
        transformStamped.transform.rotation.w);

    double roll, pitch, yaw;                       // 定义存储r\p\y的容器
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw); // 进行转换
    // std::cout << "trans roll, pitch, yaw =  \n" << roll << "  " << pitch << "  " << yaw << std::endl;

    Eigen::Matrix3d point_rotation;
    point_rotation = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
                     Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                     Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
    // std::cout << "point_rotation = " << point_rotation <<std::endl;

    trans.rotate(point_rotation);
    trans.pretranslate(Eigen::Vector3d(transformStamped.transform.translation.x,
                                       transformStamped.transform.translation.y,
                                       transformStamped.transform.translation.z));

    return gotTransform;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "relocalization"); // 节点的名字
    Scan2MapLocation scan_to_map;
    ros::MultiThreadedSpinner spinner(2); // Use 2 threads
    spinner.spin();                       // spin() will not return until the node has been shutdown
    return 0;
}
