cmake_minimum_required(VERSION 3.0.2)
project(relocalization)

add_subdirectory(3d_bbs_lib)
# #set(CMAKE_BUILD_TYPE Debug)
#set(CMAKE_BUILD_TYPE Release)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  pcl_conversions
  pcl_ros
)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# 寻找PCL的库
# find_package(PCL REQUIRED)
#find_package(Eigen3 REQUIRED)
find_package(OpenMP)
find_package(yaml-cpp REQUIRED)
if (OpenMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif ()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES scan_to_map
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
#  ${EIGEN3_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  # 3d_bbs_lib/bbs3d/include
)

#include_directories("/usr/include/eigen3")



set(COMMON_LIBRARIES
        ${PCL_LIBRARIES}
        yaml-cpp
        )

# relocalization
add_executable(relocalization src/relocalization.cpp)

if(BUILD_CUDA)
  add_definitions(-DBUILD_CUDA)
  target_link_libraries(relocalization 
  ${catkin_LIBRARIES}
  ${COMMON_LIBRARIES}
  gpu_bbs3d
  )
else()
  target_link_libraries(relocalization 
  ${catkin_LIBRARIES}
  ${COMMON_LIBRARIES}
  cpu_bbs3d
  )
endif()
#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

