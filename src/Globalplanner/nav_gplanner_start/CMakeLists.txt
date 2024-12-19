cmake_minimum_required(VERSION 3.10)

project(nav_gplanner_start)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        sensor_msgs
        pcl_ros
        geometry_msgs
        )

# find_package(Eigen3 REQUIRED)

catkin_package(
        CATKIN_DEPENDS
        roscpp
        std_msgs
        sensor_msgs
        pcl_ros
        geometry_msgs
)

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY config/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)