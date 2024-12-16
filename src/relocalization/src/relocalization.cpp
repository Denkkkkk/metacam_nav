#include "relocalization.h"
#include "std_msgs/Bool.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

std_msgs::Bool need_reloc;
int count;

Scan2MapLocation::Scan2MapLocation() : scan_resoult(new sensor_msgs::PointCloud2), private_node_("~"), tf_listener_(tfBuffer_)
{
    // \033[1;32m，\033[0m 终端显示成绿色
    ROS_INFO_STREAM("\033[1;32m---->  Relocation started.\033[0m");

    // 初始化订阅者
    laser_scan_subscriber_ = node_handle_.subscribe("/cloud_interface", 1, &Scan2MapLocation::Scan2SubmapCallback,
                                                    this, ros::TransportHints().tcpNoDelay());
    odom_subscriber_ = node_handle_.subscribe("/odom_interface", 20, &Scan2MapLocation::OdomCallback,
                                              this, ros::TransportHints().tcpNoDelay());

    scan_pointcloud_publisher_ = node_handle_.advertise<sensor_msgs::PointCloud2>("scan_pointcloud", 10);

    scan_map_publisher_ = node_handle_.advertise<sensor_msgs::PointCloud2>("scan_map", 10);

    icp_pointcloud_publisher_ = node_handle_.advertise<PointCloudT>("icp_pointcloud", 1, this);

    fgr_pointcloud_publisher_ = node_handle_.advertise<PointCloudT>("fgr_pointcloud", 1, this);

    location_publisher_ = node_handle_.advertise<geometry_msgs::PoseWithCovarianceStamped>("location_match", 1, this);

    match_point_publisher_ = node_handle_.advertise<geometry_msgs::PoseStamped>("relocalization", 1, this);

    vehicle_pose_publisher_ = node_handle_.advertise<geometry_msgs::PoseStamped>("vehicle_pose", 1, this);

    vehicle_pose_map_publisher_ = node_handle_.advertise<geometry_msgs::PoseStamped>("vehicle_pose_map", 1, this);

    need_relocal_sub = node_handle_.subscribe<std_msgs::Bool>("/need_reloc", 1, &Scan2MapLocation::needRelocCallBack, this);

    vehicle_publisher_ = node_handle_.advertise<sensor_msgs::PointCloud2>("rector_points", 1, this);

    // relocalization_srv_ = node_handle_.advertiseService("relocat_srv", &Scan2MapLocation::RelocalizeCallback, this);

    // 参数初始化
    InitParams();
    cloud_map_ = boost::shared_ptr<PointCloudT>(new PointCloudT());
    submap = boost::shared_ptr<PointCloudT>(new PointCloudT());
    submap_coarse = boost::shared_ptr<PointCloudT>(new PointCloudT());
    submap_fine = boost::shared_ptr<PointCloudT>(new PointCloudT());
    // 指针的初始化
    sensor_msgs::PointCloud2 pc2_temp;
    if (pcd_path != "-1")
    {
        pcl::io::loadPCDFile(pcd_path, pc2_temp);
        pc2_temp.header.frame_id = "map";
        pc2_temp.header.stamp = ros::Time::now();
        pcl::fromROSMsg(pc2_temp, *cloud_map_);
        pciof::pcl_to_eigen(cloud_map_, tar_points);
        map_initialized_ = true;
    }
    // 3D-BBS初始化
    Init3DBBS();
    odom_cloud_ = boost::shared_ptr<PointCloudT>(new PointCloudT());
}
Eigen::Vector3d to_eigen(const std::vector<double> &vec)
{
    Eigen::Vector3d e_vec;
    for (int i = 0; i < 3; ++i)
    {
        if (vec[i] == 6.28)
        {
            e_vec(i) = 2 * M_PI;
        }
        else
        {
            e_vec(i) = vec[i];
        }
    }
    return e_vec;
}
bool Scan2MapLocation::load_config(const std::string &config)
{
    YAML::Node conf = YAML::LoadFile(config);

    std::cout << "[YAML] Loading paths..." << std::endl;

    std::cout << "[YAML] Loading 3D-BBS parameters..." << std::endl;
    min_level_res = conf["min_level_res"].as<double>();
    max_level = conf["max_level"].as<int>();

    if (min_level_res == 0.0 || max_level == 0)
    {
        std::cout << "[ERROR] Set min_level and num_layers except for 0" << std::endl;
        return false;
    }

    std::cout << "[YAML] Loading angular search range..." << std::endl;
    std::vector<double> min_rpy_temp = conf["min_rpy"].as<std::vector<double>>();
    std::vector<double> max_rpy_temp = conf["max_rpy"].as<std::vector<double>>();
    if (min_rpy_temp.size() == 3 && max_rpy_temp.size() == 3)
    {
        min_rpy = to_eigen(min_rpy_temp);
        max_rpy = to_eigen(max_rpy_temp);
    }
    else
    {
        std::cout << "[ERROR] Set min_rpy and max_rpy correctly" << std::endl;
        return false;
    }

    std::cout << "[YAML] Loading score threshold percentage..." << std::endl;
    score_threshold_percentage = conf["score_threshold_percentage"].as<double>();

    std::cout << "[YAML] Loading downsample parameters..." << std::endl;
    tar_leaf_size = conf["tar_leaf_size"].as<float>();
    src_leaf_size = conf["src_leaf_size"].as<float>();
    min_scan_range = conf["min_scan_range"].as<double>();
    max_scan_range = conf["max_scan_range"].as<double>();

    timeout_msec = conf["timeout_msec"].as<int>();

    use_gicp = conf["use_gicp"].as<bool>();
    return true;
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
    private_node_.param<std::string>("pcd_path", pcd_path, "-1");
    private_node_.param<std::string>("config_path", config_path, "-1");

    if (!load_config(config_path))
    {
        std::cout << "[ERROR]  Loading config file failed" << std::endl;
    };
}
void Scan2MapLocation::Init3DBBS()
{
    // 3D-BBS初始化
    if (use_gicp)
    {
        gicp_ptr = std::make_unique<pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>>();
        gicp_ptr->setInputTarget(cloud_map_);
    };
    std::cout << "[Voxel map] Creating hierarchical voxel map..." << std::endl;

    auto initi_t1 = std::chrono::high_resolution_clock::now();
    bbs3d_ptr = std::make_unique<cpu::BBS3D>();
    bbs3d_ptr->set_tar_points(tar_points, min_level_res, max_level);
    bbs3d_ptr->set_trans_search_range(tar_points);
    auto init_t2 = std::chrono::high_resolution_clock::now();
    double init_time = std::chrono::duration_cast<std::chrono::nanoseconds>(init_t2 - initi_t1).count() / 1e6;
    std::cout << "[Voxel map] Execution time: " << init_time << "[msec] " << std::endl;
    // sleep for 1 second
    ros::Duration(1).sleep();

    bbs3d_ptr->set_angular_search_range(min_rpy, max_rpy);
    bbs3d_ptr->set_score_threshold_percentage(static_cast<float>(score_threshold_percentage));
    if (timeout_msec > 0)
    {
        bbs3d_ptr->enable_timeout();
        bbs3d_ptr->set_timeout_duration_in_msec(timeout_msec);
    }

    // num_threads
    int num_threads = 4;
    bbs3d_ptr->set_num_threads(num_threads);
    std::cout << "[Setting] 3DBBS location started." << std::endl;
    // 发布地图点云
    sensor_msgs::PointCloud2 map_pointcloud;
    pcl::toROSMsg(*cloud_map_, map_pointcloud);
    map_pointcloud.header.frame_id = "map";
    map_pointcloud.header.stamp = ros::Time::now();
    scan_map_publisher_.publish(map_pointcloud);
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
    vehicleRoll = roll;
    vehiclePitch = pitch;

    pub_vehicle_pose.header.frame_id = "map";
    pub_vehicle_pose.header.stamp = ros::Time::now();
    pub_vehicle_pose.pose.position.x = vehicleX;
    pub_vehicle_pose.pose.position.y = vehicleY;
    pub_vehicle_pose.pose.position.z = vehicleZ;
    pub_vehicle_pose.pose.orientation.x = geoQuat.x;
    pub_vehicle_pose.pose.orientation.y = geoQuat.y;
    pub_vehicle_pose.pose.orientation.z = geoQuat.z;
    pub_vehicle_pose.pose.orientation.w = geoQuat.w;
    Eigen::Isometry3d vehicle_pose = Eigen::Isometry3d::Identity();
    vehicle_pose.translation() = Eigen::Vector3d(vehicleX, vehicleY, vehicleZ);
    Eigen::Quaterniond q(geoQuat.w, geoQuat.x, geoQuat.y, geoQuat.z);
    vehicle_pose.linear() = q.toRotationMatrix();
    Eigen::Isometry3d vehicle_pose_map = vehicle_pose;
    pub_vehicle_pose_map.header.frame_id = "map";
    pub_vehicle_pose_map.header.stamp = ros::Time::now();
    pub_vehicle_pose_map.pose.position.x = vehicle_pose_map.translation().x();
    pub_vehicle_pose_map.pose.position.y = vehicle_pose_map.translation().y();
    pub_vehicle_pose_map.pose.position.z = vehicle_pose_map.translation().z();
    Eigen::Quaterniond q_map(vehicle_pose_map.linear());
    pub_vehicle_pose_map.pose.orientation.x = q_map.x();
    pub_vehicle_pose_map.pose.orientation.y = q_map.y();
    pub_vehicle_pose_map.pose.orientation.z = q_map.z();
    pub_vehicle_pose_map.pose.orientation.w = q_map.w();

    vehicle_pose_publisher_.publish(pub_vehicle_pose);
    vehicle_pose_map_publisher_.publish(pub_vehicle_pose_map);
    if (need_reloc.data)
    {

        pcl::PointXYZ point;
        point.x = vehicleX;
        point.y = vehicleY;
        point.z = vehicleZ;
        odom_cloud_->points.push_back(point);
        odom_cloud_->width = 1;
        odom_cloud_->height = odom_cloud_->points.size();
        odom_cloud_->is_dense = false;
        odom_filter.setInputCloud(odom_cloud_);
        odom_filter.setLeafSize(0.3, 0.3, 0.3);
        odom_filter.filter(*odom_cloud_);
        if (!reloc_active)
        {
            std::cout << "[Reloc] trajectory: " << odom_cloud_->points.size() * 0.3 << "m" << std::endl;
        }

        if (odom_cloud_->points.size() > 3 && reloc_active == false)
        {
            reloc_active = true;
            //            need_reloc.data = false;
            std::cout << "[Reloc] Relocation is active." << std::endl;
        }
    }
}
void Scan2MapLocation::Scan2SubmapCallback(const sensor_msgs::PointCloud2::ConstPtr &scan_msg)
{
    // 发布地图点云
    sensor_msgs::PointCloud2 map_pointcloud;
    pcl::toROSMsg(*cloud_map_, map_pointcloud);
    map_pointcloud.header.frame_id = "map";
    map_pointcloud.header.stamp = ros::Time::now();
    scan_map_publisher_.publish(map_pointcloud);
    if (!need_reloc.data)
    {
        return;
    }
    else
    {
        if (!reloc_active)
        {
            // transfer scan_msg to PointCloudT and add to submap
            std::cout << "[Reloc] Accumulating submap...   " << std::endl;
            PointCloudT::Ptr scan_cloud(new PointCloudT());
            pcl::fromROSMsg(*scan_msg, *scan_cloud);
            *submap += *scan_cloud;
            fine_filter.setInputCloud(submap);
            fine_filter.setLeafSize(0.1, 0.1, 0.1);
            fine_filter.filter(*submap);
            return;
        }

        else
        {
            // 输出提示
            ROS_INFO_STREAM("\033[1;32m----> Relocation started.\033[0m");
            // 判断地图和里程计数据是否初始化，如果没有则退出
            if (!map_initialized_)
            {
                ROS_WARN("map or odom not initialized");
                return;
            }

            //    pcl::fromROSMsg(*scan_msg, *submap);
            std::cout << "[Reloc] submap size: " << submap->points.size() << std::endl;
            auto initi_t1 = std::chrono::high_resolution_clock::now();
            coarse_filter.setInputCloud(submap);
            coarse_filter.setLeafSize(src_leaf_size, src_leaf_size, src_leaf_size);
            coarse_filter.filter(*submap_coarse);
            fine_filter.setInputCloud(submap);
            fine_filter.setLeafSize(0.1, 0.1, 0.1);
            fine_filter.filter(*submap_fine);
            auto init_t2 = std::chrono::high_resolution_clock::now();
            double init_time = std::chrono::duration_cast<std::chrono::nanoseconds>(init_t2 - initi_t1).count() / 1e6;
            std::cout << "[Downsample] Execution time: " << init_time << "[msec] " << std::endl;
            std::vector<Eigen::Vector3d> src_points;
            pciof::pcl_to_eigen(submap_coarse, src_points);
            bbs3d_ptr->set_src_points(src_points);
            bbs3d_ptr->localize();

            std::cout << "[Localize] Execution time: " << bbs3d_ptr->get_elapsed_time() << "[msec] " << std::endl;
            std::cout << "[Localize] Score: " << bbs3d_ptr->get_best_score() << std::endl;

            if (!bbs3d_ptr->has_localized())
            {
                if (bbs3d_ptr->has_timed_out())
                    std::cout << "[Failed] Localization timed out." << std::endl;
                else
                    std::cout << "[Failed] Score is below the threshold." << std::endl;
                return;
            }
            pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>());
            if (use_gicp)
            {
                auto gicp_t1 = std::chrono::high_resolution_clock::now();
                gicp_ptr->setInputSource(submap_fine);
                gicp_ptr->align(*output_cloud_ptr, bbs3d_ptr->get_global_pose().cast<float>());
                match_result_.matrix() = gicp_ptr->final_transformation_.cast<double>();
                auto gicp_t2 = std::chrono::high_resolution_clock::now();
                double gicp_time = std::chrono::duration_cast<std::chrono::nanoseconds>(gicp_t2 - gicp_t1).count() / 1e6;
                std::cout << "[GICP] Execution time: " << init_time << "[msec] " << std::endl;
            }
            else
            {
                pcl::transformPointCloud(*submap_fine, *output_cloud_ptr, bbs3d_ptr->get_global_pose());
                match_result_.matrix() = bbs3d_ptr->get_global_pose().cast<double>();
            }
            need_reloc.data = false;
            reloc_active = false;
            odom_cloud_->clear();
            submap->clear();
            submap_coarse->clear();
            submap_fine->clear();
            // 发布定位结果
            std::cout << "odom_interface_to_map: " << match_result_.matrix() << std::endl;
            pub_match_result.header.frame_id = "map";
            pub_match_result.header.stamp = ros::Time::now();
            pub_match_result.pose.position.x = match_result_.matrix()(0, 3);
            pub_match_result.pose.position.y = match_result_.matrix()(1, 3);
            pub_match_result.pose.position.z = match_result_.matrix()(2, 3);
            Eigen::Quaterniond q(match_result_.matrix().block<3, 3>(0, 0));
            pub_match_result.pose.orientation.x = q.x();
            pub_match_result.pose.orientation.y = q.y();
            pub_match_result.pose.orientation.z = q.z();
            pub_match_result.pose.orientation.w = q.w();
            match_point_publisher_.publish(pub_match_result);
            output_cloud_ptr->header.frame_id = "map";
            fgr_pointcloud_publisher_.publish(*output_cloud_ptr);
        }
    }
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

int main(int argc, char **argv)
{
    ros::init(argc, argv, "relocalization"); // 节点的名字
    Scan2MapLocation scan_to_map;
    ros::MultiThreadedSpinner spinner(4); // Use 4 threads
    spinner.spin();                       // spin() will not return until the node has been shutdown
    return 0;
}
