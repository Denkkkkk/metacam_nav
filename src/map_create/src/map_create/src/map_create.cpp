#include <map_create.h>

MapRecord::MapRecord() : terrain_map_record(new pcl::PointCloud<pcl::PointXYZI>()) // 参数列表来赋值
{
  nhPrivate.getParam("record", record);
  nhPrivate.getParam("leaf_size", leaf_size);
  nhPrivate.getParam("pcd_path", pcd_path);

  terrain_map_sub = nh.subscribe<sensor_msgs::PointCloud2>("/tower/mapping/cloud_colored", 5, &MapRecord::terrainMapCallback, this);
  terrain_map_pub = nh.advertise<sensor_msgs::PointCloud2>("/map_record_pub", 5);
  terrain_map_xyzi = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>()); // 如果不用参数列表，就得这样
}

void MapRecord::terrainMapCallback(const sensor_msgs::PointCloud2::ConstPtr &terrain_map_pc2)
{
  terrain_map_xyzi->clear();
  pcl::fromROSMsg(*terrain_map_pc2, *terrain_map_xyzi);
  pcl::PointXYZI point;
  for (int i = 0; i < terrain_map_xyzi->points.size(); i++)
  {
    point = terrain_map_xyzi->points[i];
    terrain_map_record->push_back(point);
  }
  // 下采样降低维度，在0.2的立方体内统一使用重心代表这个体素内所有点
  // filter point cloud with constant leaf size 0.1m
  pcl::PointCloud<pcl::PointXYZI> filter_rs_cloud;                  // 创建一个临时的PCL点云对象，用于存储下采样后的点云数据。
  terrainRecordFilter.setLeafSize(leaf_size, leaf_size, leaf_size); // 设置滤波时创建的体素体积为0.1m立方体
  terrainRecordFilter.setInputCloud(terrain_map_record);            // 传入输入点云
  terrainRecordFilter.filter(filter_rs_cloud);                      // 下采样滤波后的点云
  terrain_map_record->clear();
  *terrain_map_record = filter_rs_cloud;
  ROS_ERROR("terrainMapSize: %ld", terrain_map_record->points.size());

  sensor_msgs::PointCloud2 terrain_map_record_msg;
  pcl::toROSMsg(*terrain_map_record, terrain_map_record_msg);
  terrain_map_record_msg.header.frame_id = "map";
  terrain_map_record_msg.header.stamp = terrain_map_pc2->header.stamp;
  terrain_map_pub.publish(terrain_map_record_msg);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "terrain_map_record");
  MapRecord map_record_node;
  if (map_record_node.record)
  {
    ros::spin();
  }
  ros::Rate rate(100);
  bool status = ros::ok();
  while (status)
  {
    sensor_msgs::PointCloud2 terrain_map_record_msg; // PointCloud2适合版本低的点云文件
    pcl::io::loadPCDFile(map_record_node.pcd_path, terrain_map_record_msg);
    terrain_map_record_msg.header.frame_id = "map";
    terrain_map_record_msg.header.stamp = ros::Time::now();
    map_record_node.terrain_map_pub.publish(terrain_map_record_msg); // 发布地形点云图
    status = ros::ok();
    rate.sleep();
  }
}