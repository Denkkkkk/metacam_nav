
cmake_minimum_required(VERSION 2.8)
project(nav_serial_ctrl)

set(CMAKE_BUILD_TYPE "Debug")

set(USE_DIABLO ON)
if(NOT USE_DIABLO)
    return()
endif()

add_compile_options(-std=c++17)
add_definitions(-D_WITH_SERIAL)
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs
  diablo_sdk
)
catkin_package(
  CATKIN_DEPENDS
  roscpp
  std_msgs
  geometry_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} src/nav_serial_ctrl.cpp)

target_link_libraries(${PROJECT_NAME}
  diablo_sdk
  ${catkin_LIBRARIES}
)

# target_link_libraries(${PROJECT_NAME} -ldw -lbfd)
add_dependencies(${PROJECT_NAME} diablo_sdk diablo_sdk_generate_messages_cpp)

# ############
# # Install ##
# ############
install(
  TARGETS
  nav_serial_ctrl
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)