cmake_minimum_required(VERSION 3.10)
set(CMAKE_CXX_STANDARD 17)
project(start_pathPlan_real)

# set(CMAKE_BUILD_TYPE "Debug")
# set(CMAKE_BUILD_TYPE Release)
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=address")
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
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${catkin_INCLUDE_DIRS}
)

add_executable(start_control src/main.cpp src/start_control.cpp)
add_dependencies(start_control ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(start_control ${catkin_LIBRARIES})

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
