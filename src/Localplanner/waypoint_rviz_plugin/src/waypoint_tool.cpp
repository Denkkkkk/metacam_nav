#include "waypoint_tool.h"

namespace rviz {
    WaypointTool::WaypointTool()
    {
        shortcut_key_ = 'w';

        topic_property_ = new StringProperty("Topic", "waypoint", "The topic on which to publish navigation waypionts.",
                                             getPropertyContainer(), SLOT(updateTopic()), this);
    }

    void WaypointTool::onInitialize()
    {
        PoseTool::onInitialize();
        setName("Waypoint");
        updateTopic();
        vehicle_z = 0;
    }

    void WaypointTool::updateTopic()
    {
        sub_ = nh_.subscribe<nav_msgs::Odometry>("/Odometry", 5, &WaypointTool::odomHandler, this);
        pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/way_point", 5);
        // pub_joy_ = nh_.advertise<sensor_msgs::Joy>("/joy", 5);
    }

    void WaypointTool::odomHandler(const nav_msgs::Odometry::ConstPtr &odom)
    {
        vehicle_z = odom->pose.pose.position.z;
    }

    void WaypointTool::onPoseSet(double x, double y, double theta)
    {
        geometry_msgs::PoseStamped waypoint;
        waypoint.header.frame_id = "map";
        waypoint.header.stamp = ros::Time::now();
        waypoint.pose.position.x = x;
        waypoint.pose.position.y = y;
        waypoint.pose.position.z = vehicle_z;
        geometry_msgs::Quaternion geoQuat = tf::createQuaternionMsgFromRollPitchYaw(0, 0, theta);
        waypoint.pose.orientation.x = geoQuat.x;
        waypoint.pose.orientation.y = geoQuat.y;
        waypoint.pose.orientation.z = geoQuat.z;
        waypoint.pose.orientation.w = geoQuat.w;
        usleep(10000);
        pub_.publish(waypoint);
    }
} // namespace rviz

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz::WaypointTool, rviz::Tool)
