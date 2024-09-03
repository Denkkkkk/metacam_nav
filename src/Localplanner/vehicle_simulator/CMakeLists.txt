cmake_minimum_required(VERSION 2.8.3)
project(vehicle_simulator)

#set(CMAKE_BUILD_TYPE Release)
# #set(CMAKE_BUILD_TYPE Debug)

set(BUILD_STATIC_LIBS ON)
set(BUILD_SHARED_LIBS OFF)
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  pcl_ros
  msg_process
)
###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS
  roscpp
  std_msgs
  sensor_msgs
  pcl_ros
)
###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
  "${PROJECT_SOURCE_DIR}/include"
  /usr/local/include # Location when using 'make system_install'
  /usr/include       # More usual location (e.g. when installing using a package)
)

## Specify additional locations for library files
link_directories(
  /usr/local/lib # Location when using 'make system_install'
  /usr/lib       # More usual location (e.g. when installing using a package)
)

## Declare executables
add_executable(robotSimulator src/robotSimulator.cpp)
add_executable(car_control src/car_control.cpp)
add_executable(key_control src/key_control.cpp)
add_dependencies(key_control ${catkin_EXPORTED_TARGETS} msg_process_generate_messages_cpp)
add_dependencies(car_control ${catkin_EXPORTED_TARGETS} msg_process_generate_messages_cpp)


## Specify libraries to link a library or executable target against
target_link_libraries(robotSimulator ${catkin_LIBRARIES})
target_link_libraries(car_control ${catkin_LIBRARIES} )
target_link_libraries(key_control ${catkin_LIBRARIES} )

install(TARGETS car_control key_control robotSimulator
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY rviz/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
)
