cmake_minimum_required(VERSION 2.8.3)
project(pcl_to_laserscan)

find_package(catkin REQUIRED COMPONENTS
  laser_geometry
  message_filters
  nodelet
  roscpp
  sensor_msgs
  tf2
  tf2_ros
  tf2_sensor_msgs
  pcl_ros
  roslib
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES laserscan_to_pointcloud pcl_to_laserscan
  CATKIN_DEPENDS laser_geometry message_filters nodelet roscpp sensor_msgs tf2 tf2_ros tf2_sensor_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(pcl_to_laserscan src/pointcloud_to_laserscan_nodelet.cpp)
target_link_libraries(pcl_to_laserscan ${catkin_LIBRARIES} yaml-cpp)

add_executable(pointcloud_to_laserscan_node src/pointcloud_to_laserscan_node.cpp)
target_link_libraries(pointcloud_to_laserscan_node pcl_to_laserscan ${catkin_LIBRARIES} yaml-cpp)

install(TARGETS
  pcl_to_laserscan
  pointcloud_to_laserscan_node
	RUNTIME DESTINATION lib
	LIBRARY DESTINATION lib
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(FILES nodelets.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

if(CATKIN_ENABLE_TESTING)
  find_package(roslint REQUIRED)
  roslint_cpp()
  roslint_add_test()
endif()
