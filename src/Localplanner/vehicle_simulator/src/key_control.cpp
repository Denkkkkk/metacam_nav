/**
 * @file key_control.cpp
 * @author 李东权 (1327165187@qq.com)
 * @brief 键盘控制。
 * @version 1.0
 * @date 2024-1.18
 * @copyright Copyright (c) 2024
 *
 */
#include "msg_process/send_data.h"
#include <cerrno>
#include <cstring>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <sys/select.h>
#include <termios.h>
#include <tf/tf.h>
#include <thread>
#include <unistd.h>

int kfd = 0;
char key;
double roll, pitch, yaw;
double odomTime = 0;
double speed = 0.02;
geometry_msgs::Twist twist_stamped_msg;
struct termios cooked, raw;
msg_process::send_data serial_send_datas; // 其他模式发送给串口的信息
ros::Publisher serial_pub;
ros::Publisher twistStamped_pub;
std::string mode = "real";

void detect_keyboard(); // 按键检测
void init_keyboard();   // 初始化键盘

/**
 * @brief 键盘映射规则
 */
void key_mapping(char key_);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "key_control");
    ros::NodeHandle nh;
    ros::NodeHandle nhPrivate = ros::NodeHandle("~");
    nhPrivate.getParam("mode", mode);
    // 模式切换
    if (mode == "virture")
        twistStamped_pub = nh.advertise<geometry_msgs::Twist>("TITA/cmd_vel", 1);
    else if (mode == "real")
    {
        serial_pub = nh.advertise<msg_process::send_data>("/serial_send_data", 10);
        twistStamped_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    }
    else
    {
        ROS_ERROR("mode error");
        return -1;
    }
    if (mode == "real")
    {
        std::thread pub_thread([&]() {
            while (ros::ok())
            {
                ros::Rate rate_thread(50);
                serial_pub.publish(serial_send_datas);
            }
        });
    }
    ros::Rate rate(20);
    init_keyboard(); // 初始化键盘
    ROS_INFO("Keyboard teleop started.\nUse 'w' to move forward,\n's' to move backward,\n'a' to move left,\n'd'to move right,\nany other(such as 'o') to stop\n and 'q' to spin.");
    while (ros::ok())
    {
        detect_keyboard(); // 键盘检测
        key_mapping(key);
        twistStamped_pub.publish(twist_stamped_msg);
        ROS_WARN("vx:%f,vy:%f,w:%f", twist_stamped_msg.linear.x, twist_stamped_msg.linear.y, twist_stamped_msg.angular.z);
        rate.sleep();
    }
    // 恢复标准输入属性
    tcsetattr(kfd, TCSANOW, &cooked);

    return 0;
}

void detect_keyboard()
{
    if (read(kfd, &key, 1) < 0)
    {
        perror("read():");
        ROS_ERROR("read() failed: %s", strerror(errno));
        exit(-1);
    }
}

void init_keyboard()
{
    // 获取标准输入的文件描述符
    tcgetattr(kfd, &cooked);                       // 获取标准输入的属性
    memcpy(&raw, &cooked, sizeof(struct termios)); // 复制一份cooked的配置
    raw.c_lflag &= ~(ICANON | ECHO);               // 关闭标准输入的标准模式，使输入字符不需要回车确认
    raw.c_cc[VEOL] = 1;                            // 有些系统下不设置会出现字符打印问题
    tcsetattr(kfd, TCSANOW, &raw);                 // 设置属性
}

void key_mapping(char key_)
{
    // 仿真环境
    if (mode == "virture")
    {
        switch (key_)
        {
        case 'w':
            twist_stamped_msg.linear.x += speed;
            twist_stamped_msg.linear.y = 0;
            twist_stamped_msg.angular.z = 0;
            break;
        case 's':
            twist_stamped_msg.linear.x -= speed;
            twist_stamped_msg.linear.y = 0;
            twist_stamped_msg.angular.z = 0;
            break;
        case 'a':
            twist_stamped_msg.linear.x = 0;
            twist_stamped_msg.linear.y += speed;
            twist_stamped_msg.angular.z = 0;
            break;
        case 'd':
            twist_stamped_msg.linear.x = 0;
            twist_stamped_msg.linear.y -= speed;
            twist_stamped_msg.angular.z = 0.0;
            break;
        case 'q':
            twist_stamped_msg.linear.x = 0.0;
            twist_stamped_msg.linear.y = 0.0;
            twist_stamped_msg.angular.z += 0.01;
            break;
        case 'e':
            twist_stamped_msg.linear.x = 0.0;
            twist_stamped_msg.linear.y = 0.0;
            twist_stamped_msg.angular.z -= 0.01;
            break;
        default:
            twist_stamped_msg.linear.x = 0.0;
            twist_stamped_msg.linear.y = 0.0;
            twist_stamped_msg.angular.z = 0.0;
            // ROS_ERROR("default!");
        }
    }
    else
    {
        switch (key_)
        {
        case 'w':
            twist_stamped_msg.linear.x += speed;
            twist_stamped_msg.linear.y = 0;
            twist_stamped_msg.angular.z = 0;
            break;
        case 's':
            twist_stamped_msg.linear.x -= speed;
            twist_stamped_msg.linear.y = 0;
            twist_stamped_msg.angular.z = 0;
            break;
        case 'a':
            twist_stamped_msg.linear.x = 0;
            twist_stamped_msg.linear.y += speed;
            twist_stamped_msg.angular.z = 0;
            break;
        case 'd':
            twist_stamped_msg.linear.x = 0;
            twist_stamped_msg.linear.y -= speed;
            twist_stamped_msg.angular.z = 0;
            break;
        case 'q':
            twist_stamped_msg.linear.x = 0.0;
            twist_stamped_msg.linear.y = 0.0;
            twist_stamped_msg.angular.z += speed;
            break;
        case 'e':
            twist_stamped_msg.linear.x = 0.0;
            twist_stamped_msg.linear.y = 0.0;
            twist_stamped_msg.angular.z -= speed;
            break;
        default:
            twist_stamped_msg.linear.x = 0.0;
            twist_stamped_msg.linear.y = 0.0;
            twist_stamped_msg.angular.z = 0.0;
        }
    }
}