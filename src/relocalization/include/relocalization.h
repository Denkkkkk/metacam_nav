#include <chrono>
#include <cmath>
#include <deque>
#include <iostream>
#include <mutex>
#include <thread>
#include <typeinfo>
#include <vector>
#include <yaml-cpp/yaml.h>
#include <cpu_bbs3d/bbs3d.hpp>
#include <gpu_bbs3d/bbs3d.cuh>

// ros
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>

// tf2
#include "tf2_ros/transform_broadcaster.h"
#include <tf/transform_listener.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/utils.h>
#include <tf2_ros/transform_listener.h>
// pcl
#include <pcl/io/pcd_io.h> //PCD文件输入输出操作
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pointcloud_iof/pcl_eigen_converter.hpp>
#include <pointcloud_iof/pcd_loader.hpp>
#include <pcl/point_cloud.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <message_filters/subscriber.h>
#include <pcl/point_representation.h>
//----------------
#include <boost/make_shared.hpp>
#include <pcl/console/time.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/features/fpfh.h>
#include <pcl/features/fpfh_omp.h> //包含fpfh加速计算的omp多核并行计算
#include <pcl/features/normal_3d.h>

#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_ros/transforms.h>

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "std_msgs/Bool.h"
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <pcl/registration/gicp.h>
#include <tf/transform_broadcaster.h>

class Scan2MapLocation
{

private:
    ros::NodeHandle node_handle_;            // ros中的句柄
    ros::NodeHandle private_node_;           // ros中的私有句柄
    ros::Subscriber laser_scan_subscriber_;  // 声明一个Subscriber
    ros::Subscriber map_subscriber_;         // 声明一个Subscriber
    ros::Subscriber initialpose_subscriber_; // 声明一个Subscriber
    ros::Subscriber odom_subscriber_;
    ros::Subscriber need_relocal_sub;

    ros::Publisher scan_pointcloud_publisher_;
    ros::Publisher icp_pointcloud_publisher_;
    ros::Publisher fgr_pointcloud_publisher_;
    ros::Publisher location_publisher_;
    ros::Publisher match_point_publisher_;
    ros::Publisher vehicle_pose_publisher_;
    ros::Publisher vehicle_pose_map_publisher_;
    ros::Publisher scan_map_publisher_;

    ros::Publisher vehicle_publisher_;


    geometry_msgs::PoseWithCovarianceStamped location_match; // 定位结果
    geometry_msgs::PoseStamped pub_match_result;             // 定位结果
    geometry_msgs::PoseStamped pub_vehicle_pose;                 // 车辆位置
    geometry_msgs::PoseStamped pub_vehicle_pose_map;             // 车辆位置

    tf2_ros::Buffer tfBuffer_;
    tf2_ros::TransformListener tf_listener_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;


    Eigen::Isometry3d map_to_base_ = Eigen::Isometry3d::Identity();       // map到base的欧式变换矩阵4x4
    Eigen::Isometry3d match_result_ = Eigen::Isometry3d::Identity();      // icp匹配结果
    Eigen::Isometry3d last_match_result_ = Eigen::Isometry3d::Identity(); // 上一帧icp匹配结果

    // parameters
    bool map_initialized_ = false;
    bool scan_initialized_ = false;
    bool odom_initialized_ = false;

    bool save_pcd_ = false;

    std::string odom_frame_;
    std::string base_frame_;
    std::string map_frame_;
    std::string lidar_frame_;

    double vehicleX;
    double vehicleY;
    double vehicleZ;
    double vehicleYaw;
    double vehicleRoll;
    double vehiclePitch;


    // 用于odom获取坐标变换
    std::mutex odom_lock_;
    std::deque<nav_msgs::Odometry> odom_queue_;
    int odom_queue_length_;


    std::vector<double> location_restricted_zone_;

    double Variance_X; // 协方差
    double Variance_Y;
    double Variance_Yaw;
    double obstacleHeightThre = 0.01;
    std::string pcd_path;
    std::string config_path;
    tf::TransformListener listener; // tf监听器

    // pcl
    typedef pcl::PointXYZ PointT;
    typedef pcl::PointCloud<pcl::PointXYZ> PointCloudT;

    // 初始化fgr算法对象
    PointCloudT::Ptr cloud_map_;
    std::vector<Eigen::Vector3f> tar_points;
    PointCloudT::Ptr cloud_scan_;
    PointCloudT::Ptr submap;
    PointCloudT::Ptr submap_coarse;
    PointCloudT::Ptr submap_fine;
    pcl::VoxelGrid<pcl::PointXYZ> coarse_filter;
    pcl::VoxelGrid<pcl::PointXYZ> fine_filter;

    // 3D-BBS parameters
    std::unique_ptr<gpu::BBS3D> bbs3d_ptr;
    std::unique_ptr<pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>> gicp_ptr;
    double min_level_res;
    int max_level;
    // angular search range
    Eigen::Vector3d min_rpy;
    Eigen::Vector3d max_rpy;

    // score threshold percentage
    double score_threshold_percentage;

    // downsample
    float tar_leaf_size, src_leaf_size;
    double min_scan_range, max_scan_range;

    // timeout
    int timeout_msec;

    // align
    bool use_gicp;

    bool reloc_active = false;
    PointCloudT::Ptr odom_cloud_;
    pcl::VoxelGrid<pcl::PointXYZ> odom_filter;


    Eigen::Matrix4f init_transform = Eigen::Matrix4f::Identity();
    sensor_msgs::PointCloud2::Ptr scan_resoult;



    void InitParams();
    void Init3DBBS();

public:
    Scan2MapLocation();
    bool load_config(const std::string& config);
    void Scan2SubmapCallback(const sensor_msgs::PointCloud2::ConstPtr &scan_msg);
    void OdomCallback(const nav_msgs::Odometry::ConstPtr &odometryMsg);
    void needRelocCallBack(const std_msgs::Bool::ConstPtr &need_status);

    bool is_coordinate_in_range(const std::vector<double> &vec, const Eigen::Isometry3d &coord);
};