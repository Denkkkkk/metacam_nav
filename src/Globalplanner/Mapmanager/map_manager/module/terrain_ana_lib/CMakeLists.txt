cmake_minimum_required(VERSION 2.8.3)
project(terrain_ana_lib)

set(BUILD_STATIC_LIBS OFF)
set(BUILD_SHARED_LIBS ON)
# set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  pcl_ros
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
#   ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  include
  /usr/local/include # Location when using 'make system_install'
  /usr/include       # More usual location (e.g. when installing using a package)
)

## Specify additional locations for library files
link_directories(
  /usr/local/lib # Location when using 'make system_install'
  /usr/lib       # More usual location (e.g. when installing using a package)
)

## Declare executables
add_library(terrain_ana_lib src/terrain_ana.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(terrain_ana_lib 
    ${catkin_LIBRARIES} 
    # ${OpenCV_LIBRARIES} 
    ${PCL_LIBRARIES}
)

install(TARGETS terrain_ana_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)