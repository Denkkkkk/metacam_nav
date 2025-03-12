#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

class OdomToMappingInit
{
public:
    OdomToMappingInit()
    {
        // 订阅/Odometry话题
        odom_subscriber_ = nh_.subscribe("/Odometry", 10, &OdomToMappingInit::odomCallback, this);

        // 初始化tf2广播器
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>();
    }

    /**
     * @brief 里程计回调函数
     * @note 里程计数据作为变换发布，父坐标系是mapping_init，子坐标系是body
     */
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        // 创建变换消息
        geometry_msgs::TransformStamped transformStamped;

        // 填充变换数据
        transformStamped.header.stamp = ros::Time::now();
        transformStamped.header.frame_id = "mapping_init";  // 里程计相对于这个坐标系
        transformStamped.child_frame_id = "body";  // 发布到这个坐标系

        // 从里程计消息中获取位置和方向
        transformStamped.transform.translation.x = msg->pose.pose.position.x;
        transformStamped.transform.translation.y = msg->pose.pose.position.y;
        transformStamped.transform.translation.z = msg->pose.pose.position.z;

        transformStamped.transform.rotation = msg->pose.pose.orientation;

        // 发布变换
        tf_broadcaster_->sendTransform(transformStamped);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber odom_subscriber_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "odom_tf");
    OdomToMappingInit odom_to_mapping_init;

    // 循环处理ROS回调函数
    ros::spin();

    return 0;
}
