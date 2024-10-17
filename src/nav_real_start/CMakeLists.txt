cmake_minimum_required(VERSION 3.10)
set(CMAKE_CXX_STANDARD 17)
project(nav_real_start)

# set(CMAKE_BUILD_TYPE "Debug")
# set(CMAKE_BUILD_TYPE Release)
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=address")
find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
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

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
