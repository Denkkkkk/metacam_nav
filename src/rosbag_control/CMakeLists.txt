cmake_minimum_required(VERSION 3.0.2)
project(rosbag_control)
find_package(yaml-cpp REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  nav_msgs
  std_srvs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# add_executable(rosbag_control src/rosbag_control.cpp)
# target_link_libraries(rosbag_control
#   ${catkin_LIBRARIES}
#   yaml-cpp
# )

# install(
#         TARGETS
#         rosbag_control
#         ARCHIVE DESTINATION lib
#         LIBRARY DESTINATION lib
#         RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# install(DIRECTORY launch/
#         DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
# )