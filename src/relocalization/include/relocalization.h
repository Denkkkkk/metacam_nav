#include <chrono>
#include <cmath>
#include <deque>
#include <iostream>
#include <mutex>
#include <thread>
#include <typeinfo>
#include <vector>

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

#include "msg_process/receive_data.h"

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
    ros::Publisher removal_pointcloud_publisher_;
    ros::Publisher icp_pointcloud_publisher_;
    ros::Publisher rotate_pointcloud_publisher_;
    ros::Publisher location_publisher_;
    ros::Publisher relocate_tranform_visuial_publisher_;
    ros::Publisher match_point_publisher_;
    ros::Publisher scan_map_publisher_;

    ros::Publisher vehicle_publisher_;
    ros::Publisher safetyStop_publisher_;

    ros::ServiceServer relocalization_srv_; // 重定位服务

    geometry_msgs::PoseWithCovarianceStamped location_match; // 定位结果
    geometry_msgs::PoseStamped pub_match_result;             // 定位结果

    tf2_ros::Buffer tfBuffer_;
    tf2_ros::TransformListener tf_listener_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;

    tf2::Transform base_in_odom_;          // base_link在odom坐标系下的坐标
    tf2::Transform base_in_odom_keyframe_; // base_link在odom坐标系下的keyframe的坐标

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

    // 用于计算时间
    std::chrono::steady_clock::time_point scan_start_time_;
    std::chrono::steady_clock::time_point scan_end_time_;
    std::chrono::steady_clock::time_point scan_last_end_time_;
    std::chrono::duration<double> scan_time_used_;

    std::chrono::steady_clock::time_point start_time_;
    std::chrono::steady_clock::time_point end_time_;
    std::chrono::duration<double> time_used_;

    double vehicleX;
    double vehicleY;
    double vehicleZ;
    double vehicleYaw;

    double match_time_;          // 当前匹配的时间
    double last_match_time_ = 0; // 上一帧匹配的时间

    double scan_time_; // 当前用于匹配的雷达数据时间
    float best_score;
    double globle_x;
    double globle_y;
    double globle_yaw;
    double sector_angel1;
    double sector_angel2;
    double sector_angel3;
    double sector_angel4;

    // 用于odom获取坐标变换
    std::mutex odom_lock_;
    std::deque<nav_msgs::Odometry> odom_queue_;
    int odom_queue_length_;

    // relocation
    double Relocation_Weight_Score_;
    double Relocation_Weight_Distance_;
    double Relocation_Weight_Yaw_;
    double Relocation_Maximum_Iterations_;
    double Relocation_Score_Threshold_Max_;

    std::vector<double> location_restricted_zone_;

    double Variance_X; // 协方差
    double Variance_Y;
    double Variance_Yaw;
    double obstacleHeightThre = 0.01;
    std::string pcd_path;
    tf::TransformListener listener; // tf监听器

    // pcl
    typedef pcl::PointXYZ PointT;
    typedef pcl::PointCloud<pcl::PointXYZ> PointCloudT;

    // 初始化icp算法对象
    PointCloudT::Ptr cloud_map_;
    PointCloudT::Ptr cloud_scan_;
    Eigen::Matrix4f init_transform = Eigen::Matrix4f::Identity();
    sensor_msgs::PointCloud2::Ptr scan_resoult;
    std_msgs::Bool need_stop;

    void InitParams();
    // 坐标变换
    bool Get2TimeTransform(Eigen::Isometry3d &trans);

public:
    Scan2MapLocation();
    void ScanCallback(const sensor_msgs::PointCloud2::ConstPtr &scan_msg);
    void OdomCallback(const nav_msgs::Odometry::ConstPtr &odometryMsg);
    void needRelocCallBack(const std_msgs::Bool::ConstPtr &need_status);

    void rotatePointCloud(PointCloudT::Ptr &cloud_msg, const Eigen::Affine3f &rotation, const Eigen::Affine3f &robo_pose);
    bool is_coordinate_in_range(const std::vector<double> &vec, const Eigen::Isometry3d &coord);
    void Filter(const sensor_msgs::PointCloud2::Ptr &scan_resoult, PointCloudT::Ptr filtered_cloud);
    geometry_msgs::PoseWithCovarianceStamped Isometry3d_to_PoseWithCovarianceStamped(const Eigen::Isometry3d &iso);
    void gicp_registration(Eigen::Isometry3d &trans, PointCloudT::Ptr source, PointCloudT::Ptr target, const Eigen::Isometry3d &robot_pose);
    bool GetTransform(Eigen::Isometry3d &trans, const std::string parent_frame, const std::string child_frame, const ros::Time stamp);
};