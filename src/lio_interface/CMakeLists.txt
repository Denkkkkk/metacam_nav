cmake_minimum_required(VERSION 3.10)

project(lio_interface)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        sensor_msgs
        pcl_ros
        )

catkin_package(
        CATKIN_DEPENDS
        roscpp
        std_msgs
        sensor_msgs
        pcl_ros
)

include_directories(${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR} include)

add_executable(odom_interface src/odom_interface.cpp)
add_executable(cloud_interface src/cloud_interface.cpp)

target_link_libraries(odom_interface ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${EIGEN3_LIBS})
target_link_libraries(cloud_interface ${catkin_LIBRARIES})

install(
  TARGETS
  odom_interface cloud_interface
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)