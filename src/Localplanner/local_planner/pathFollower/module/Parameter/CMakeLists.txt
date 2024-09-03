find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

add_library(pathFollower_param src/parameters.cpp)
target_include_directories(pathFollower_param PUBLIC ${catkin_INCLUDE_DIRS} include)
target_link_libraries(pathFollower_param ${catkin_LIBRARIES})

install(
  TARGETS
  pathFollower_param
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)