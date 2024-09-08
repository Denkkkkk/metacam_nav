/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/

#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <math.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <string.h>
#include <unistd.h>

using namespace UNITREE_LEGGED_SDK;

class Custom
{
public:
    Custom(uint8_t level) : safe(LeggedType::Go1),
                            udp(level, 8090, "192.168.19.10", 8082)
    {
        udp.InitCmdData(cmd);
        sub_cmd_vel = nh.subscribe<geometry_msgs::Twist>("/cmd_vel", 1, &cmd_vel_callback, this);
        sub_test_string = nh.subscribe<std_msgs::String>("/test_string", 1, &test_string_callback, this);
    }
    void UDPRecv();
    void UDPSend();
    void RobotControl();
    void cmd_vel_callback(const geometry_msgs::Twist::ConstPtr &msg);
    void test_string_callback(const std_msgs::String::ConstPtr &msg);

    Safety safe;
    UDP udp;
    HighCmd cmd = {0};
    HighState state = {0};
    int motiontime = 0;
    float dt = 0.01; // 0.001~0.01
    ros::NodeHandle nh;
    ros::Subscriber sub_cmd_vel;
    ros::Subscriber sub_test_string;
};

void Custom::cmd_vel_callback(const geometry_msgs::Twist::ConstPtr &msg)
{
    udp.GetRecv(state);
    //   printf("%d   %f\n", motiontime, state.imu.quaternion[2]);
    cmd.mode = 0; // 0:idle, default stand      1:forced stand     2:walk continuously
    cmd.gaitType = 0;
    cmd.speedLevel = 0;
    cmd.footRaiseHeight = 0;
    cmd.bodyHeight = 0;
    cmd.velocity[0] = msg->linear.x;
    cmd.velocity[1] = msg->linear.y;
    cmd.yawSpeed = msg->angular.z;

    udp.SetSend(cmd);
}

void Custom::test_string_callback(const std_msgs::String::ConstPtr &msg)
{
    for (const char &c : msg->data)
    {
        switch (c)
        {
        case 'w':
            cmd.mode = 2;
            cmd.gaitType = 1;
            cmd.velocity[0] = 0.2f;
            cmd.bodyHeight = 0.1;
            break;
        case 'a':
            cmd.mode = 2;
            cmd.gaitType = 1;
            cmd.velocity[1] = 0.2f;
            cmd.bodyHeight = 0.1;
            break;
        case 's':
            cmd.mode = 2;
            cmd.gaitType = 1;
            cmd.velocity[0] = -0.2f;
            cmd.bodyHeight = 0.1;
            break;
        case 'd':
            cmd.mode = 2;
            cmd.gaitType = 1;
            cmd.velocity[1] = -0.2f;
            cmd.bodyHeight = 0.1;
            break;
        case 'z':
            cmd.mode = 0;
            break;
        case 'x':
            cmd.mode = 1;
            break;
        case 'c':
            cmd.mode = 2;
            break;

        case 'v':
            cmd.mode = 3;
            break;
        case 'b':
            cmd.mode = 4;
            break;
        case 'n':
            cmd.mode = 5;
            break;
        case 'm':
            cmd.mode = 6;
            break;
        default:
            cmd.mode = 0;     // 0:idle, default stand      1:forced stand     2:walk continuously
            cmd.gaitType = 0; // 行走类型，0:行走 1:奔跑 2:攀爬
            cmd.speedLevel = 0;
            cmd.footRaiseHeight = 0;
            cmd.bodyHeight = 0;
            cmd.euler[0] = 0;
            cmd.euler[1] = 0;
            cmd.euler[2] = 0;
            cmd.velocity[0] = 0.0f;
            cmd.velocity[1] = 0.0f;
            cmd.yawSpeed = 0.0f;
            cmd.reserve = 0;
        }
        ros::Duration(1.0).sleep();
    }
}

void Custom::UDPRecv()
{
    udp.Recv();
}

void Custom::UDPSend()
{
    udp.Send();
}

void Custom::RobotControl()
{
    motiontime += 2;
    udp.GetRecv(state);
    //   printf("%d   %f\n", motiontime, state.imu.quaternion[2]);
    cmd.mode = 0;     // 0:idle, default stand      1:forced stand     2:walk continuously
    cmd.gaitType = 0; // 行走类型，0:行走 1:奔跑 2:攀爬
    cmd.speedLevel = 0;
    cmd.footRaiseHeight = 0;
    cmd.bodyHeight = 0;
    cmd.euler[0] = 0;
    cmd.euler[1] = 0;
    cmd.euler[2] = 0;
    cmd.velocity[0] = 0.0f;
    cmd.velocity[1] = 0.0f;
    cmd.yawSpeed = 0.0f;
    cmd.reserve = 0;

    if (motiontime > 0 && motiontime < 1000)
    {
        cmd.mode = 1;
        cmd.euler[0] = -0.3;
    }
    if (motiontime > 1000 && motiontime < 2000)
    {
        cmd.mode = 1;
        cmd.euler[0] = 0.3;
    }
    if (motiontime > 2000 && motiontime < 3000)
    {
        cmd.mode = 1;
        cmd.euler[1] = -0.2;
    }
    if (motiontime > 3000 && motiontime < 4000)
    {
        cmd.mode = 1;
        cmd.euler[1] = 0.2;
    }
    if (motiontime > 4000 && motiontime < 5000)
    {
        cmd.mode = 1;
        cmd.euler[2] = -0.2;
    }
    if (motiontime > 5000 && motiontime < 6000)
    {
        cmd.mode = 1;
        cmd.euler[2] = 0.2;
    }
    if (motiontime > 6000 && motiontime < 7000)
    {
        cmd.mode = 1;
        cmd.bodyHeight = -0.2;
    }
    if (motiontime > 7000 && motiontime < 8000)
    {
        cmd.mode = 1;
        cmd.bodyHeight = 0.1;
    }
    if (motiontime > 8000 && motiontime < 9000)
    {
        cmd.mode = 1;
        cmd.bodyHeight = 0.0;
    }
    if (motiontime > 9000 && motiontime < 11000)
    {
        cmd.mode = 5;
    }
    if (motiontime > 11000 && motiontime < 13000)
    {
        cmd.mode = 7;
    }
    if (motiontime > 13000 && motiontime < 15000)
    {
        cmd.mode = 6;
    }
    if (motiontime > 15000 && motiontime < 16000)
    {
        cmd.mode = 0;
    }
    if (motiontime > 16000 && motiontime < 20000)
    {
        cmd.mode = 2;
        cmd.gaitType = 2;
        cmd.velocity[0] = 0.4f;
        cmd.yawSpeed = 2;
        cmd.footRaiseHeight = 0.1;
    }
    if (motiontime > 20000 && motiontime < 22000)
    {
        cmd.mode = 0;
        cmd.velocity[0] = 0;
    }
    if (motiontime > 22000 && motiontime < 26000)
    {
        cmd.mode = 2;
        cmd.gaitType = 1;
        cmd.velocity[0] = 0.2f;
        cmd.bodyHeight = 0.1;
    }

    // straightHand mode usage
    if (motiontime > 26000 && motiontime < 27000)
    {
        cmd.mode = 1;
    }
    if (motiontime > 27000 && motiontime < 35000)
    {
        cmd.mode = 11;
    }

    // jumpYaw mode usage
    if (motiontime > 35000 && motiontime < 36000)
    {
        cmd.mode = 1;
    }
    if (motiontime > 36000 && motiontime < 37000)
    {
        cmd.mode = 10;
    }

    udp.SetSend(cmd);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "unitree_nav");
    ros::NodeHandle nh;

    Custom custom(HIGHLEVEL);
    // 设置定时器运行函数，每隔custom.dt即0.01秒运行一次（100hz）
    // LoopFunc loop_control("control_loop", custom.dt, boost::bind(&Custom::RobotControl, &custom));
    LoopFunc loop_udpSend("udp_send", custom.dt, 3, boost::bind(&Custom::UDPSend, &custom));
    LoopFunc loop_udpRecv("udp_recv", custom.dt, 3, boost::bind(&Custom::UDPRecv, &custom));

    loop_udpSend.start();
    loop_udpRecv.start();
    // loop_control.start();
    ROS_INFO("Unitree_nav node get into spin().");
    ros::Rate rate(100);
    while (ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
