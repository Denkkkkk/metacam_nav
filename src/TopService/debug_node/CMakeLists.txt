cmake_minimum_required(VERSION 3.0.2)
project(debug_node)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        geometry_msgs
        std_srvs
        nav_msgs
        roslib
)

catkin_package(
        CATKIN_DEPENDS
        roscpp
        std_msgs
        std_srvs
        nav_msgs
)

include_directories(
        include
        /usr/local/include # Location when using 'make system_install'
        /usr/include # More usual location (e.g. when installing using a package)
        ${catkin_INCLUDE_DIRS}
)
link_directories(
  /usr/local/lib # Location when using 'make system_install'
  /usr/lib # More usual location (e.g. when installing using a package)
)

add_executable(debug_node src/debug_node.cpp)
target_link_libraries(debug_node ${catkin_LIBRARIES} nav_spd_log)