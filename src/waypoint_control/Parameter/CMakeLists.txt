find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  roslib
)

add_library(waypoint_control_param src/parameters.cpp)

target_include_directories(waypoint_control_param PUBLIC ${catkin_INCLUDE_DIRS} include)
target_link_libraries(waypoint_control_param ${catkin_LIBRARIES} yaml-cpp)

install(
  TARGETS
  waypoint_control_param
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)