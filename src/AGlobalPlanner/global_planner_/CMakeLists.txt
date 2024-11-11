cmake_minimum_required(VERSION 3.10)

project(global_planner_)

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

include_directories(${catkin_INCLUDE_DIRS}  ${EIGEN3_INCLUDE_DIR} include)

add_executable(global_planner_ src/global_planner.cpp)

target_link_libraries(global_planner_ ${catkin_LIBRARIES}  ${EIGEN3_LIBS})

