find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  msg_process
  lplanner_reconfigure
)

aux_source_directory(src DIR_SRCS)
add_library(LpNode SHARED ${DIR_SRCS})

target_include_directories(LpNode PUBLIC include ${catkin_INCLUDE_DIRS})
add_dependencies(LpNode ${catkin_EXPORTED_TARGETS} lplanner_reconfigure_generate_messages_cpp)
target_link_libraries(LpNode Parts ${catkin_LIBRARIES} localPlanner_param)
