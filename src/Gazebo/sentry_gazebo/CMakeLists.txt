if(NOT BUILD_SIMULATOR)
    return()
endif()

cmake_minimum_required(VERSION 2.8.3)
project(sentry_gazebo)
find_package(catkin REQUIRED)
catkin_package()

install(DIRECTORY launch/ 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
install(DIRECTORY urdf/ 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf
)
install(DIRECTORY worlds/ 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/worlds
)
install(DIRECTORY meshes/ 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/meshes
)
