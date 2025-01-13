find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        nav_msgs
        std_srvs
)
set(CMAKE_CXX_STANDARD 17)

#spd_logging INTERFACE 类型的库不编译任何源代码，只提供头文件、宏定义、链接选项等。
add_library(spd_logging  INTERFACE)
target_link_libraries(spd_logging
        INTERFACE
        ${catkin_LIBRARIES}
)

target_include_directories(spd_logging INTERFACE 
      include 
      ${catkin_INCLUDE_DIRS}
      )

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