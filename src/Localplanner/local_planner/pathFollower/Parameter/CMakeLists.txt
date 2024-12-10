find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

add_library(pathFollower_param src/parameters.cpp)

target_include_directories(pathFollower_param PUBLIC ${catkin_INCLUDE_DIRS} include)
target_link_libraries(pathFollower_param ${catkin_LIBRARIES} yaml-cpp)

install(
  TARGETS
  pathFollower_param
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)