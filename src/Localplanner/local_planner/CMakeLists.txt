cmake_minimum_required(VERSION 3.0.2)
project(local_planner)
add_subdirectory(localPlanner)
add_subdirectory(pathFollower)
set(CMAKE_CXX_STANDARD 17)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  pcl_ros
)

# ## Find catkin macros and libraries
catkin_package(
  CATKIN_DEPENDS
  roscpp
  std_msgs
  sensor_msgs
  pcl_ros
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  /usr/local/include # Location when using 'make system_install'
  /usr/include # More usual location (e.g. when installing using a package)
  pathFollower/include
)
link_directories(
  /usr/local/lib # Location when using 'make system_install'
  /usr/lib # More usual location (e.g. when installing using a package)
)

add_executable(localPlanner localPlanner/src/local_main.cpp)
add_dependencies(localPlanner ${catkin_EXPORTED_TARGETS})
target_link_libraries(localPlanner LpNode ${PCL_LIBRARIES} ${catkin_LIBRARIES} nav_spd_log)

add_executable(pathFollower
  pathFollower/src/pathFollower.cpp
  pathFollower/src/call_back.cpp
)

# 为编译添加msssages包的依赖
add_dependencies(pathFollower ${catkin_EXPORTED_TARGETS})
target_link_libraries(pathFollower ${catkin_LIBRARIES} ${PCL_LIBRARIES} pathFollower_param nav_spd_log)

install(
  TARGETS
  localPlanner pathFollower
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

install(DIRECTORY localPlanner/paths/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/localPlanner/paths
)