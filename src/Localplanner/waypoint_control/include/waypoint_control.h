#include <math.h>
#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include "extern.h"
#include "midpoint.h"
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h> //pcd 读写类相关的头文件。
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>
#include <std_srvs/Empty.h>
#include <tf/transform_datatypes.h>

enum ControlMode
{
    GOALDIRECT,
    MIDPOINT,
    RECOVER,
};

enum ControlModePub
{
    pubGOALDIRECT,
    pubMIDPlanning,
    pubGUIDEPlanning,
    pubRECOVER,
};

class waypoint_control
{
public:
    waypoint_control();
    ~waypoint_control() {};
    /**
     * @brief 主函数
     *
     */
    void run();

private:
    ros::NodeHandle nh;
    ros::NodeHandle nhPrivate = ros::NodeHandle("~");
    ros::Subscriber subGoalPoint;
    ros::Subscriber subGoalPath;
    ros::Subscriber subPathStatus;
    ros::Subscriber subOdometry;
    ros::Publisher pubWayPoint;
    ros::Publisher pubControlMode;
    ros::Publisher pubCloseMap;
    ros::ServiceClient clearCostmap_client;

    void goalPointCallback(const geometry_msgs::PoseStamped::ConstPtr &point);
    void goalPathCallback(const nav_msgs::Path::ConstPtr &pathIn);
    void pathStatusCallback(const std_msgs::Bool::ConstPtr &status);
    void odometryCallback(const nav_msgs::Odometry::ConstPtr &odom);
    void publishWayPoint(geometry_msgs::PoseStamped &point);
    void updateCase();
    void cancelLateraling();
    void updateDuration();

    std::unique_ptr<MidPointMode> _midP_mode_ptr;

    int pathPointID = 0;
    // 路径预处理
    double endDis;
    ControlMode control_mode = GOALDIRECT;
    ControlModePub control_mode_pub = pubGOALDIRECT;
    std_msgs::Bool path_status;

    double point_begin = -1.0;
    double point_end = -1.0;
    double point_duration = 0;
    double point_distance_time = 0; // 根据距离远近计算的所需时间
    bool point_time_out = false;    // mid_point超时
    std_srvs::Empty srv;
    double goalX_last;
    double goalY_last;

    // 恢复模式参数
    bool is_lateraling = false;    // 是否正在横移
    double lateral_dis = 0.8;      // 尝试跳出卡死位置的后移距离
    double lateral_time = 2.0;     // 横移持续时间(秒)
    double lateral_goal_dis = 1.0; // 超过目标距离才横移
    double lateral_begin = -1.0;   // 横移开始时间
    double lateral_end = -1.0;     // 横移结束时间
    double lateral_duration = 0;   // 横移持续时间
    double update_begin_dis;       // 更新横移开始时间的前后点距离

    /* 跟随全局规划参数 */
    geometry_msgs::PoseStamped goal_point;
    bool need_pub_goal = false;
    double vehicle_goal_dis;
    bool pathNew = false;
    double pathState_update_time;
    double update_begin_dis_goalPoint;
    double update_begin_dis_vehicleDis;

    // 动态参数
    double default_point_time = 6.0; // waypoint持续基础时间
    double reach_goal_dis = 0.5;
    bool goal_point_only = false;
    bool clear_justnow = false;
    bool debug = false;
    bool use_recovery_mode = false;
};