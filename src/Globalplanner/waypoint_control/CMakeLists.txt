cmake_minimum_required(VERSION 2.8.3)
project(waypoint_control)

#set(CMAKE_BUILD_TYPE Debug)
# #set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 17)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  pcl_ros
  std_srvs
)
catkin_package(
  CATKIN_DEPENDS
  roscpp
  std_msgs
  sensor_msgs
  pcl_ros
)
include_directories(
  ${catkin_INCLUDE_DIRS}
  # ${PCL_INCLUDE_DIRS}
  include
  ${PROJECT_SOURCE_DIR}/include
  /usr/local/include # Location when using 'make system_install'
  /usr/include       # More usual location (e.g. when installing using a package)
)

## Specify additional locations for library files
link_directories(
  /usr/local/lib # Location when using 'make system_install'
  /usr/lib       # More usual location (e.g. when installing using a package)
)

aux_source_directory(src SRC_LIST)
add_executable(waypoint_control ${SRC_LIST})
target_link_libraries(waypoint_control ${catkin_LIBRARIES})


install(TARGETS  waypoint_control 
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY rviz/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
)


