cmake_minimum_required(VERSION 3.10)

project(msg_process)

set(CMAKE_CXX_STANDARD  14)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        nav_msgs
        geometry_msgs
        message_generation
        )

add_message_files(
        DIRECTORY msg
)


generate_messages(
        DEPENDENCIES
        std_msgs
        geometry_msgs
        nav_msgs
)

catkin_package(
        CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
)
