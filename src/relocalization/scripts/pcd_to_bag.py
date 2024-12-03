#!/usr/bin/env python

import os
import rospy
import rosbag
import numpy as np
import open3d as o3d
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header
from nav_msgs.msg import Odometry


def read_odom_file(odom_file):
    """读取 odom.txt 文件中的位姿信息"""
    odom_data = []
    with open(odom_file, 'r') as f:
        for line in f:
            values = line.strip().split()
            timestamp = int(values[0])  # 时间戳
            x, y, z = map(float, values[1:4])  # 平移
            qx, qy, qz, qw = map(float, values[4:8])  # 四元数
            odom_data.append((timestamp, x, y, z, qx, qy, qz, qw))
    return odom_data


def load_pcd_as_pointcloud2(pcd_file, ros_time, frame_id="map"):
    """加载 .pcd 文件并转换为 PointCloud2 消息"""
    # 使用 open3d 加载点云
    pcd = o3d.io.read_point_cloud(pcd_file)
    points = np.asarray(pcd.points)  # 转换为 numpy 数组

    # 创建 PointCloud2 消息
    header = Header()
    header.stamp = ros_time
    header.frame_id = frame_id
    point_cloud_msg = pc2.create_cloud_xyz32(header, points)

    return point_cloud_msg


def create_rosbag(odom_file, pcd_folder, output_bag):
    """生成 ROS bag 文件"""
    # 读取 odom 数据
    odom_data = read_odom_file(odom_file)

    # 创建 rosbag
    bag = rosbag.Bag(output_bag, 'w')

    try:
        for idx, (timestamp, x, y, z, qx, qy, qz, qw) in enumerate(odom_data):
            # 转换时间戳为 ROS 时间
            ros_time = rospy.Time.from_sec(timestamp * 1e-9)

            # 创建 Odometry 消息
            odom = Odometry()
            odom.header.stamp = ros_time
            odom.header.frame_id = "map"  # 父坐标系
            odom.child_frame_id = "base_link"  # 子坐标系（根据需要修改）

            # 设置位置
            odom.pose.pose.position.x = x
            odom.pose.pose.position.y = y
            odom.pose.pose.position.z = z

            # 设置姿态（四元数）
            odom.pose.pose.orientation.x = qx
            odom.pose.pose.orientation.y = qy
            odom.pose.pose.orientation.z = qz
            odom.pose.pose.orientation.w = qw

            # 如果需要，可以设置线速度和角速度（此处留空）
            odom.twist.twist.linear.x = 0.0
            odom.twist.twist.linear.y = 0.0
            odom.twist.twist.linear.z = 0.0
            odom.twist.twist.angular.x = 0.0
            odom.twist.twist.angular.y = 0.0
            odom.twist.twist.angular.z = 0.0

            # 写入位姿消息到 rosbag
            bag.write("/odom_interface", odom, ros_time)

            # 加载对应的点云文件
            pcd_file = os.path.join(pcd_folder, "{:06d}.pcd".format(idx))
            if os.path.exists(pcd_file):
                # 使用 open3d 加载点云并转换为 PointCloud2 消息
                point_cloud_msg = load_pcd_as_pointcloud2(pcd_file, ros_time)

                # 写入点云消息到 rosbag
                bag.write("/cloud_interface", point_cloud_msg, ros_time)
                print("Write to rosbag: {}".format(pcd_file))

            else:
                rospy.logwarn("PointCloud file not found: {}".format(pcd_file))

    finally:
        bag.close()
        rospy.loginfo("ROS bag file created: {}".format(output_bag))


if __name__ == "__main__":
    # 初始化 ROS 节点
    rospy.init_node("create_rosbag")

    # 配置文件路径
    # 替换为你的 odom.txt 路径
    odom_file = "/home/bob/datasets/Skyland/Registration/xbotpark-raw/4F_000/pose/odom.txt"
    pcd_folder = "/home/bob/datasets/Skyland/Registration/xbotpark-raw/4F_000/scans"  # 替换为点云文件夹路径
    output_bag = "/home/bob/datasets/Skyland/Registration/xbotpark-raw/4F_000/rosbag/05m/output.bag"  # 替换为输出 bag 文件路径

    # 创建 rosbag
    create_rosbag(odom_file, pcd_folder, output_bag)
