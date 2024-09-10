cmake_minimum_required(VERSION 3.10)

project(map_to_odom)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  pcl_ros
  geometry_msgs
)

# find_package(Eigen3 REQUIRED)
catkin_package(
  CATKIN_DEPENDS
  roscpp
  std_msgs
  sensor_msgs
  pcl_ros
  geometry_msgs
)

include_directories(${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR} include)

add_executable(map_to_odom src/map_to_odom.cpp)
add_dependencies(map_to_odom ${catkin_EXPORTED_TARGETS})
target_link_libraries(map_to_odom ${catkin_LIBRARIES} ${EIGEN3_LIBS})

install(
  TARGETS
  map_to_odom
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
