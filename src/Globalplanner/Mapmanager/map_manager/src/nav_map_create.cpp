#include <nav_map_create.h>

MapRecord::MapRecord() : terrain_map_record(new pcl::PointCloud<pcl::PointXYZI>()) // 参数列表来赋值
{
    nhPrivate.getParam("record", record);
    nhPrivate.getParam("leaf_size", leaf_size);
    nhPrivate.getParam("pcd_path", pcd_path);
    nhPrivate.getParam("visual_pcd_path", visual_pcd_path);
    nhPrivate.getParam("obstacleHeightThre", obstacleHeightThre);
    nhPrivate.getParam("save_always", save_always);

    terrain_map_sub = nh.subscribe<sensor_msgs::PointCloud2>("terrain_map", 5, &MapRecord::terrainMapCallback, this);   // 订阅点云话题： terrain_map: 地形点云图
    // / terrain_map话题是从Localplanner的terrain_analysis输出的地形点云数据
    // TODO 解耦lio
    cloud_map_sub = nh.subscribe<sensor_msgs::PointCloud2>("/cloud_interface", 5, &MapRecord::cloudMapCallback, this);  // 订阅点云话题 cloud_interface: cloud点云图
    // /cloud_interface话题是从lio输出的点云数据

    terrain_map_pub = nh.advertise<sensor_msgs::PointCloud2>("/terrain_map_pub", 5);
    cloud_map_pub = nh.advertise<sensor_msgs::PointCloud2>("/cloud_map_pub", 5);
    map_flie_pub = nh.advertise<sensor_msgs::PointCloud2>("/map_file_pub", 1, true);
    terrain_map_xyzi = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>()); // 如果不用参数列表，就得这样
    cloud_map_record = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());

    terrainRecordFilter.setLeafSize(leaf_size, leaf_size, leaf_size); // 设置滤波时创建的体素体积为0.1m立方体

    // 获取当前时间
    auto now = std::chrono::system_clock::now();
    std::time_t raw_time = std::chrono::system_clock::to_time_t(now);
    // 转换为当地时间
    std::tm *time_info = std::localtime(&raw_time);
    // 使用字符串构造文件名，格式为 YYYYMMDD_HHMM
    terrain_map_ss << pcd_path << "terrain" << std::put_time(time_info, "%Y%m%d_%H_%M") << ".pcd";
    terrain_map_ss_temp << pcd_path << "pcd_temp/" << "terrain" << std::put_time(time_info, "%Y%m%d_%H_%M") << ".pcd";
    relocalization_map_ss << pcd_path << "relocal" << std::put_time(time_info, "%Y%m%d_%H_%M") << ".pcd";
    relocalization_map_ss_temp << pcd_path << "pcd_temp/" << "relocal" << std::put_time(time_info, "%Y%m%d_%H_%M") << ".pcd";
}

/**
 * @brief 保存点云数据到PCD文件
 * @param cloud 点云数据
 * @param file_name 文件名
 * @param binary_ 是否以二进制格式保存
 */
void MapRecord::save_pcd(const sensor_msgs::PointCloud2 &cloud, std::stringstream &file_name, bool binary_)
{
    std::stringstream ss;
    if (!file_name.str().empty())
    {
        ss << file_name.str();
    }
    else
    {
        // 获取当前时间
        auto now = std::chrono::system_clock::now();
        std::time_t raw_time = std::chrono::system_clock::to_time_t(now);
        // 转换为当地时间
        std::tm *time_info = std::localtime(&raw_time);
        // 使用字符串流构造文件名，格式为 YYYYMMDD_HHMM
        ss << "temp" << std::put_time(time_info, "%Y%m%d_%H_%M") << ".pcd";
    }
    ROS_INFO("Data saved to %s", ss.str().c_str());

    Eigen::Vector4f v = Eigen::Vector4f::Zero();
    Eigen::Quaternionf q = Eigen::Quaternionf::Identity();

    pcl::io::savePCDFile(ss.str(), cloud, v, q, binary_);
}

/**
 * @brief 点云回调函数
 * @note 接收点云数据，进行滤波和下采样处理，发布并保存处理后的点云数据
 */
void MapRecord::cloudMapCallback(const sensor_msgs::PointCloud2::ConstPtr &cloud_map_pc2)
{
    pcl::PointCloud<pcl::PointXYZI> cloud_map_xyzi;
    pcl::fromROSMsg(*cloud_map_pc2, cloud_map_xyzi);
    pcl::PointXYZI point;
    // 过滤噪点
    for (int i = 0; i < cloud_map_xyzi.points.size(); i++)
    {
        point = cloud_map_xyzi.points[i];
        // 非法点检查
        // nan表示不是一个数字，inf表示无穷大
        if (std::isnan(point.x) || std::isnan(point.y) || std::isnan(point.z) ||
            std::isinf(point.x) || std::isinf(point.y) || std::isinf(point.z))
        {
            continue; // 跳过非法点
        }
        cloud_map_record->push_back(point);
    }
    // 下采样降低维度，在0.1的立方体内统一使用重心代表这个体素内所有点
    // filter point cloud with constant leaf size 0.1m
    pcl::VoxelGrid<pcl::PointXYZI> cloudRecordFilter;               // 生成滤波器
    cloudRecordFilter.setLeafSize(leaf_size, leaf_size, leaf_size); // 设置滤波时创建的体素体积为0.1m立方体
    pcl::PointCloud<pcl::PointXYZI> filter_rs_cloud;                // 创建一个临时的PCL点云对象，用于存储下采样后的点云数据。
    cloudRecordFilter.setInputCloud(cloud_map_record);              // 传入输入点云
    cloudRecordFilter.filter(filter_rs_cloud);                      // 下采样滤波后的点云
    cloud_map_record->clear();
    *cloud_map_record = filter_rs_cloud;
    ROS_WARN("cloudMapSize: %ld", cloud_map_record->points.size());

    sensor_msgs::PointCloud2 cloud_map_record_msg;
    pcl::toROSMsg(*cloud_map_record, cloud_map_record_msg);
    cloud_map_record_msg.header.frame_id = "map";
    cloud_map_record_msg.header.stamp = cloud_map_pc2->header.stamp;
    cloud_map_pub.publish(cloud_map_record_msg);
    if (save_always)
    {
        static bool save_temp = true;
        CREATE_STATIC();
        if (save_temp)
        {
            save_pcd(cloud_map_record_msg, relocalization_map_ss_temp, false);
            save_temp = false;
        }
        else
        {
            save_pcd(cloud_map_record_msg, relocalization_map_ss, false);
            save_temp = true;
        }
    }
}

void MapRecord::terrainMapCallback(const sensor_msgs::PointCloud2::ConstPtr &terrain_map_pc2)
{
    terrain_map_xyzi->clear();
    pcl::fromROSMsg(*terrain_map_pc2, *terrain_map_xyzi);
    // 根据点云的强度值进行过滤，强度值大于obstacleHeightThre的点会被保留
    pcl::PointXYZI point;
    for (int i = 0; i < terrain_map_xyzi->points.size(); i++)
    {
        point = terrain_map_xyzi->points[i];
        // 点的强度值（intensity）通常表示该点的反射强度或亮度
        // 在激光雷达（LiDAR）数据中，强度值表示激光束从物体表面反射回传感器的强度。
        // 不同材质和表面的反射特性不同，因此强度值可以用来区分不同类型的物体或表面。
        // 低强度点可能来自灰尘、雨雪或远距离噪声，可通过阈值过滤。
        if (point.intensity > obstacleHeightThre)
        {
            point.intensity += 0.5;
            terrain_map_record->push_back(point);
        }
    }
    // 下采样降低维度，在0.2的立方体内统一使用重心代表这个体素内所有点
    // filter point cloud with constant leaf size 0.1m
    pcl::PointCloud<pcl::PointXYZI> filter_rs_cloud;       // 创建一个临时的PCL点云对象，用于存储下采样后的点云数据。
    terrainRecordFilter.setInputCloud(terrain_map_record); // 传入输入点云
    terrainRecordFilter.filter(filter_rs_cloud);           // 下采样滤波后的点云
    terrain_map_record->clear();
    *terrain_map_record = filter_rs_cloud;
    ROS_WARN("terrainMapSize: %ld", terrain_map_record->points.size());

    // 发布降采样后的点云terrain_map_record
    sensor_msgs::PointCloud2 terrain_map_record_msg;
    pcl::toROSMsg(*terrain_map_record, terrain_map_record_msg);
    terrain_map_record_msg.header.frame_id = "map";
    terrain_map_record_msg.header.stamp = terrain_map_pc2->header.stamp;
    terrain_map_pub.publish(terrain_map_record_msg);
    // 保存点云数据到PCD文件
    if (save_always)
    {
        static bool save_temp = true;
        CREATE_STATIC();
        // 交叉保存，相当于做了数据冗余
        if (save_temp)
        {
            save_pcd(terrain_map_record_msg, terrain_map_ss_temp, false);
            save_temp = false;
        }
        else
        {
            save_pcd(terrain_map_record_msg, terrain_map_ss, false);
            save_temp = true;
        }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "terrain_map_record", ros::init_options::AnonymousName); // 节点匿名化，AnonymousName 会在节点名称后添加一个随机的唯一后缀，这样即使你有多个同名的节点，也不会发生冲突
    MapRecord map_record_node;
    ros::Rate rate(10);
    bool status = ros::ok();
    while (status)
    {
        if (map_record_node.record)
        {
            ros::spinOnce();
        }
        sensor_msgs::PointCloud2 terrain_map_record_msg; // PointCloud2适合版本低的点云文件
        pcl::io::loadPCDFile(map_record_node.visual_pcd_path, terrain_map_record_msg);
        terrain_map_record_msg.header.frame_id = "map";
        terrain_map_record_msg.header.stamp = ros::Time::now();
        map_record_node.map_flie_pub.publish(terrain_map_record_msg); // 发布地形点云图
        status = ros::ok();
        rate.sleep();
    }
}