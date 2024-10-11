if(NOT BUILD_SIMULATOR)
    return()
endif()

cmake_minimum_required(VERSION 2.8.3)
project(sentry_gazebo)
find_package(catkin REQUIRED)
catkin_package()
