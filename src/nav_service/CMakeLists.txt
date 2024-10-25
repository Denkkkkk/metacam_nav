cmake_minimum_required(VERSION 3.10)
project(nav_service)

set(CMAKE_CXX_SIMULATE 17)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        geometry_msgs
        std_srvs
        nav_msgs
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
        ${catkin_INCLUDE_DIRS}
)

add_executable(nav_service src/nav_service.cpp)
target_link_libraries(nav_service ${catkin_LIBRARIES})

install(
        TARGETS
        nav_service
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)