cmake_minimum_required(VERSION 3.0.2)
project(lplanner_reconfigure)
set(CMAKE_CXX_STANDARD 11)
find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  roscpp
  std_msgs
  message_generation
)

# 生成msg消息文件
add_message_files(
        DIRECTORY msg
)
generate_messages(
        DEPENDENCIES
        std_msgs
)
#generate_dynamic_reconfigure_options() 必须放到catkin_package()之前
generate_dynamic_reconfigure_options(
        cfg/lpdata.cfg
)

catkin_package(
    CATKIN_DEPENDS 
    dynamic_reconfigure 
    roscpp 
    std_msgs 
    message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(lpdata_config src/lpdata_config.cpp)
add_dependencies(lpdata_config ${PROJECT_NAME}_gencfg ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
target_link_libraries(lpdata_config ${catkin_LIBRARIES})

