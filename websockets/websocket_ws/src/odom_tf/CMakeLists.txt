cmake_minimum_required(VERSION 3.10)

project(odom_tf)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  tf2_ros
)

catkin_package(
  CATKIN_DEPENDS
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
)

add_executable(odom_tf src/odom_tf.cpp)
target_link_libraries(odom_tf ${catkin_LIBRARIES})
target_include_directories(odom_tf PUBLIC ${catkin_INCLUDE_DIRS})

install(
  TARGETS
  odom_tf
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
