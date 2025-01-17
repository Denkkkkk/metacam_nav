cmake_minimum_required(VERSION 3.10)

project(websockets)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
)

set(PYTHON_EXECUTABLE "/usr/bin/python3")

catkin_python_setup()

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)


install(DIRECTORY scripts/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts
)