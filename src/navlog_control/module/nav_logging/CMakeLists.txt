find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        nav_msgs
        std_srvs    
)

#nav_normal_log INTERFACE 类型的库不编译任何源代码，只提供头文件、宏定义、链接选项等。
add_library(nav_normal_log  INTERFACE)
target_link_libraries(nav_normal_log
        INTERFACE
        ${catkin_LIBRARIES}
)

target_include_directories(nav_normal_log INTERFACE
     include 
     ${catkin_INCLUDE_DIRS}
    )

install(
  TARGETS
  nav_normal_log
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)       