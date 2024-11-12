/**
 * @file global_planner.cpp
 * @author 李东权 (1327165187@qq.com)
 * @brief 
 * @version 1.0
 * @date 2024-04-16
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#include "global_planner.h"

map_to_odom::map_to_odom()
{
    pubVehicleToMapPose = nh.advertise<geometry_msgs::PoseStamped>("/vehicleToMapPose", 5);
    pubOdomToMapPose = nh.advertise<geometry_msgs::PoseStamped>("/odomToMapPose", 5);
    pubvehicleToOdom = nh.advertise<geometry_msgs::PoseStamped>("/vehicleToOdom", 5);

    subReLocal = nh.subscribe<geometry_msgs::PoseStamped>("/relocalization", 1, &map_to_odom::reLocalizationCallBack, this);
    geometry_msgs::Quaternion geoQuat = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0);
    map_to_odom_trans.pose.orientation = geoQuat;
    map_to_odom_trans.pose.position.x = 0;
    map_to_odom_trans.pose.position.y = 0;
    map_to_odom_trans.pose.position.z = 0;
}
/**
 * @brief 已知vehicle到odom的变换，和vehicle到map的变换，计算odom到map的变换
 *
 */
void map_to_odom::reLocalizationCallBack(const geometry_msgs::PoseStamped::ConstPtr &vTm_msg)
{
    geometry_msgs::Quaternion geoQuat = vTm_msg->pose.orientation;
    if (fabs((geoQuat.x * geoQuat.x + geoQuat.y * geoQuat.y + geoQuat.z * geoQuat.z + geoQuat.w * geoQuat.w) - 1) > 0.01)
    {
        ROS_WARN("map_to_odom.cpp: received incorrect orientation!%f\n", fabs((geoQuat.x * geoQuat.x + geoQuat.y * geoQuat.y + geoQuat.z * geoQuat.z + geoQuat.w * geoQuat.w) - 1));
        return;
    }

    // 监听vehicle到odom的坐标变换
    Eigen::Isometry3d vehicle_to_odom = Eigen::Isometry3d::Identity();
    try
    {
        listener.lookupTransform("Sentry/odom", "Sentry/vehicle", ros::Time(0), transform);
    }
    catch (tf::TransformException ex)
    {
        ROS_WARN("map_to_odom.cpp: %s", ex.what());
        ros::Duration(1).sleep();
        return;
    }
    double roll_vTo, pitch_vTo, yaw_vTo;
    tf::Matrix3x3(transform.getRotation()).getEulerYPR(yaw_vTo, pitch_vTo, roll_vTo);
    Eigen::Matrix3d vehicle_to_odom_rotation;
    vehicle_to_odom_rotation = Eigen::AngleAxisd(yaw_vTo, Eigen::Vector3d::UnitZ()) *
                               Eigen::AngleAxisd(pitch_vTo, Eigen::Vector3d::UnitY()) *
                               Eigen::AngleAxisd(roll_vTo, Eigen::Vector3d::UnitX());
    vehicle_to_odom.rotate(vehicle_to_odom_rotation);
    vehicle_to_odom.pretranslate(Eigen::Vector3d(transform.getOrigin().getX(),
                                                 transform.getOrigin().getY(),
                                                 transform.getOrigin().getZ()));

    // 获取想要的vehicle到map的变换
    Eigen::Isometry3d vehicle_to_map = Eigen::Isometry3d::Identity();
    double roll_vTm, pitch_vTm, yaw_vTm;
    tf::Matrix3x3(tf::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w)).getEulerYPR(yaw_vTm, pitch_vTm, roll_vTm);
    Eigen::Matrix3d vehicle_to_map_rotation;
    vehicle_to_map_rotation = Eigen::AngleAxisd(yaw_vTm, Eigen::Vector3d::UnitZ()) *
                              Eigen::AngleAxisd(pitch_vTm, Eigen::Vector3d::UnitY()) *
                              Eigen::AngleAxisd(roll_vTm, Eigen::Vector3d::UnitX());
    vehicle_to_map.rotate(vehicle_to_map_rotation);
    vehicle_to_map.pretranslate(Eigen::Vector3d(vTm_msg->pose.position.x,
                                                vTm_msg->pose.position.y,
                                                transform.getOrigin().getZ())); // 保持z轴不变

    // 计算odom到map的坐标变换
    Eigen::Isometry3d odom_to_map = vehicle_to_odom.inverse() * vehicle_to_map;
    Eigen::Vector3d translationVector = odom_to_map.translation();
    map_to_odom_trans.pose.position.x = translationVector.x();
    map_to_odom_trans.pose.position.y = translationVector.y();
    map_to_odom_trans.pose.position.z = translationVector.z();
    Eigen::Quaterniond quat = Eigen::Quaterniond(odom_to_map.rotation());
    map_to_odom_trans.pose.orientation.x = quat.x();
    map_to_odom_trans.pose.orientation.y = quat.y();
    map_to_odom_trans.pose.orientation.z = quat.z();
    map_to_odom_trans.pose.orientation.w = quat.w();

    // 发布想要的vehicle到map的坐标变换
    wantVehicleToMap.header.stamp = ros::Time::now();
    wantVehicleToMap.header.frame_id = "map";
    wantVehicleToMap.pose = vTm_msg->pose;
    pubVehicleToMapPose.publish(wantVehicleToMap);

    // 发布订阅到的vehicle到odom的坐标变换
    geometry_msgs::Quaternion quat1 = tf::createQuaternionMsgFromRollPitchYaw(roll_vTo, pitch_vTo, yaw_vTo);
    vehicleToOdom.header.stamp = ros::Time::now();
    vehicleToOdom.header.frame_id = "map";
    vehicleToOdom.pose.position.x = transform.getOrigin().getX();
    vehicleToOdom.pose.position.y = transform.getOrigin().getY();
    vehicleToOdom.pose.position.z = transform.getOrigin().getZ();
    vehicleToOdom.pose.orientation.x = quat1.x;
    vehicleToOdom.pose.orientation.y = quat1.y;
    vehicleToOdom.pose.orientation.z = quat1.z;
    vehicleToOdom.pose.orientation.w = quat1.w;
    pubvehicleToOdom.publish(vehicleToOdom);

    // 发布计算得到的odom到map的坐标变换
    countOdomToMap.header.stamp = ros::Time::now();
    countOdomToMap.header.frame_id = "map";
    countOdomToMap.pose = map_to_odom_trans.pose;
    pubOdomToMapPose.publish(countOdomToMap);

    std::cout << "------------------------------------------------------------------" << std::endl;
    std::cout << "Isometry3d vehicle_to_map:" << std::endl;
    std::cout << vehicle_to_map.matrix() << std::endl;

    std::cout << "Isometry3d vehicle_to_odom:" << std::endl;
    std::cout << vehicle_to_odom.matrix() << std::endl;

    std::cout << "Isometry3d odom_to_map:" << std::endl;
    std::cout << odom_to_map.matrix() << std::endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "map_to_odom");
    map_to_odom map_to_odom;
    tf::TransformBroadcaster tfBroadcaster; // tf坐标变换广播器
    tf::StampedTransform odomTrans;         // tf坐标变换
    ros::Duration(1).sleep();
    ros::Rate rate(10);
    bool status = ros::ok();
    while (status)
    {
        ros::spinOnce();
        // 根据位姿信息发布tf坐标变换
        odomTrans.stamp_ = ros::Time::now();
        odomTrans.frame_id_ = "map";
        odomTrans.child_frame_id_ = "Sentry/odom";
        geometry_msgs::PoseStamped map_to_odom_trans1 = map_to_odom.map_to_odom_trans;
        geometry_msgs::Quaternion geoQuat1 = map_to_odom_trans1.pose.orientation;
        odomTrans.setRotation(tf::Quaternion(geoQuat1.x, geoQuat1.y, geoQuat1.z, geoQuat1.w));
        odomTrans.setOrigin(tf::Vector3(map_to_odom_trans1.pose.position.x, map_to_odom_trans1.pose.position.y, map_to_odom_trans1.pose.position.z));
        tfBroadcaster.sendTransform(odomTrans);
        status = ros::ok();
        rate.sleep();
    }
    return 0;
}
