/**
 * @file odom_interface.cpp
 * @author 李东权 1327165187@qq.com
 * @brief 里程计输出中间件
 * @version 0.1
 * @date 2024-04-30
 *
 * @copyright Copyright (c) 2024
 *
 */
#include "lio_interface/odom_interface.h"

OdomInterface::OdomInterface()
{
    nhPrivate.param<std::string>("ns", ns, "");

    odom_in_topic = ns + "/Odometry";
    odom_out_topic = ns + "/odom_interface";
    subOdom = nh.subscribe<nav_msgs::Odometry>(odom_in_topic, 5, &OdomInterface::odomCallBack, this);
    pubOdom = nh.advertise<nav_msgs::Odometry>(odom_out_topic, 5);
}

/**
 * @brief 里程计回调函数
 *
 * @param odom
 * @note 里程计数据转到map坐标系下
 */
void OdomInterface::odomCallBack(const nav_msgs::Odometry::ConstPtr &odom)
{
    // 坐标系转到全局map下
    odomData = *odom;
    tf::StampedTransform transform;
    transform.stamp_ = ros::Time();
    try
    {
        // ros::Time(0)表示查询最接近当前时间的变换
        listener.lookupTransform("map", "vehicle", ros::Time(0), transform);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("odom_interface:%s", ex.what());
        ros::Duration(0.5).sleep();
        return;
    }
    double roll, pitch, yaw;
    tf::Matrix3x3(transform.getRotation()).getEulerYPR(yaw, pitch, roll);
    // 里程计的yaw转到-PI to PI
    while (yaw > M_PI || yaw < -M_PI)
    {
        if (yaw > M_PI)
            yaw -= 2 * M_PI;
        else if (yaw < -M_PI)
            yaw += 2 * M_PI;
    }
    geometry_msgs::Quaternion geoQuat = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
    // 启动后短时间内补偿里程计姿态（imu会由重力计识别）
    odomData.pose.pose.orientation = geoQuat;
    odomData.pose.pose.position.x = transform.getOrigin().getX();
    odomData.pose.pose.position.y = transform.getOrigin().getY();
    odomData.pose.pose.position.z = transform.getOrigin().getZ();
    odomData.header.frame_id = "map";
    odomData.child_frame_id = "vehicle";
    pubOdom.publish(odomData);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lio_interface");
    OdomInterface odom_interface;
    ros::spin();
    return 0;
}
