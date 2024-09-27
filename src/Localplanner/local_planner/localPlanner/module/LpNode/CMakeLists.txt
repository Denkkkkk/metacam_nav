find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
)

aux_source_directory(src DIR_SRCS)
add_library(LpNode SHARED ${DIR_SRCS})

target_include_directories(LpNode PUBLIC include ${catkin_INCLUDE_DIRS})
target_link_libraries(LpNode Parts ${catkin_LIBRARIES} localPlanner_param)

install(
  TARGETS
    LpNode
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)