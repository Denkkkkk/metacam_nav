cmake_minimum_required(VERSION 3.0.2)
project(navlog_control)
find_package(yaml-cpp REQUIRED)
find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        nav_msgs
        std_srvs
)
set(CMAKE_CXX_STANDARD 11)

catkin_package()

include_directories(
        ${catkin_INCLUDE_DIRS}
        include
)

#INTERFACE 类型的库不编译任何源代码，只提供头文件、宏定义、链接选项等。
add_library(nav_logging  INTERFACE)
target_link_libraries(nav_logging
        INTERFACE
        ${catkin_LIBRARIES}
)

add_library(spd_logging SHARED
        src/spd_logging.cpp
)
target_link_libraries(spd_logging
        ${catkin_LIBRARIES}
)

add_executable(navlog_control src/navlog_control.cpp)
target_link_libraries(navlog_control
        ${catkin_LIBRARIES}
        yaml-cpp
        nav_logging
        spd_logging
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

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)