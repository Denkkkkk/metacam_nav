find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

add_library(localPlanner_param src/parameters.cpp)
target_include_directories(localPlanner_param PUBLIC ${catkin_INCLUDE_DIRS} include)
target_link_libraries(localPlanner_param ${catkin_LIBRARIES})