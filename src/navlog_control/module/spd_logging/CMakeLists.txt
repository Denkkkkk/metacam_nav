find_package(catkin REQUIRED COMPONENTS
#        roscpp
        roslib
)
set(CMAKE_CXX_STANDARD 17)

#nav_spd_log INTERFACE 类型的库不编译任何源代码，只提供头文件、宏定义、链接选项等。
add_library(nav_spd_log  INTERFACE)
target_link_libraries(nav_spd_log
        INTERFACE
        ${catkin_LIBRARIES}
)

target_include_directories(nav_spd_log INTERFACE 
      include 
      ${catkin_INCLUDE_DIRS}
      )

install(
  TARGETS
  nav_spd_log
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}/
)