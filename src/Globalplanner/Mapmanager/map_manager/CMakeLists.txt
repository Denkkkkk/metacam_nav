cmake_minimum_required(VERSION 2.8.3)
project(map_manager)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  pcl_ros
  tf2_eigen
)

# find_package(PCL REQUIRED)

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
  "${PROJECT_SOURCE_DIR}/include"
  /usr/local/include # Location when using 'make system_install'
  /usr/include       # More usual location (e.g. when installing using a package)
)

## Specify additional locations for library files
link_directories(
  /usr/local/lib # Location when using 'make system_install'
  /usr/lib       # More usual location (e.g. when installing using a package)
)

add_executable(nav_map_create src/nav_map_create.cpp)
target_link_libraries(nav_map_create ${catkin_LIBRARIES})

install(TARGETS nav_map_create
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY pcd/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/pcd
)