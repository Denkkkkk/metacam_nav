cmake_minimum_required(VERSION 3.10)
set(CMAKE_CXX_STANDARD 17)
project(map_manager)

add_subdirectory(module)
find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        geometry_msgs
        pcl_conversions
        pcl_ros
        )

catkin_package(
        CATKIN_DEPENDS
        roscpp
        std_msgs
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

add_executable(map_manager src/map_manager.cpp )
target_link_libraries(map_manager ${catkin_LIBRARIES} )

