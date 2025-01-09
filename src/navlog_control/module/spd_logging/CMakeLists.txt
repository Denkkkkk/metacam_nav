find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        nav_msgs
        std_srvs
)
set(CMAKE_CXX_STANDARD 17)
aux_source_directory(src DIR_SRCS)
add_library(spd_logging SHARED ${DIR_SRCS})

target_include_directories(spd_logging PUBLIC include ${catkin_INCLUDE_DIRS})
target_link_libraries(spd_logging ${catkin_LIBRARIES})

install(
  TARGETS
    spd_logging
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}/
)