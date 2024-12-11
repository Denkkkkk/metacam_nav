cmake_minimum_required(VERSION 3.0.2)
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
        /usr/local/include # Location when using 'make system_install'
        /usr/include # More usual location (e.g. when installing using a package)
        ${catkin_INCLUDE_DIRS}
)
link_directories(
  /usr/local/lib # Location when using 'make system_install'
  /usr/lib # More usual location (e.g. when installing using a package)
)


add_executable(nav_service src/nav_service.cpp)
add_dependencies(nav_service ${catkin_EXPORTED_TARGETS})
target_link_libraries(nav_service ${catkin_LIBRARIES} nav_service_param)

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