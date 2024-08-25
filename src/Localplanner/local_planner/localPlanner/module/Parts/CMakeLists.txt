find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
)

add_library(Parts SHARED src/Parts.cpp)
target_include_directories(Parts PUBLIC include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS}) 
target_link_libraries(Parts ${PCL_LIBRARIES} ${catkin_LIBRARIES} )