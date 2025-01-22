cmake_minimum_required(VERSION 3.0.2)
project(navlog_control)
add_subdirectory(module)
find_package(yaml-cpp REQUIRED)
find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        nav_msgs
        std_srvs
)
set(CMAKE_CXX_STANDARD 17)

catkin_package()

add_executable(navlog_control src/navlog_control.cpp)
target_link_libraries(navlog_control
        ${catkin_LIBRARIES}
        yaml-cpp
        nav_normal_log
        nav_spd_log
)

install(
     TARGETS
     navlog_control
     ARCHIVE DESTINATION lib
     LIBRARY DESTINATION lib
     RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
     DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
