find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  roslib
)

add_library(nav_service_param src/parameters.cpp)
target_include_directories(nav_service_param PUBLIC ${catkin_INCLUDE_DIRS} include)
target_link_libraries(nav_service_param ${catkin_LIBRARIES} yaml-cpp)

install(
  TARGETS
  nav_service_param
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)