cmake_minimum_required(VERSION 3.10)
project(nav_service)

add_subdirectory(Parameter)
find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        geometry_msgs
        std_srvs
        nav_msgs
        tf
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
        ${EIGEN3_INCLUDE_DIR}
)

add_executable(nav_service src/nav_service.cpp)
target_link_libraries(nav_service ${catkin_LIBRARIES} ${EIGEN3_LIBS} nav_service_param)

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