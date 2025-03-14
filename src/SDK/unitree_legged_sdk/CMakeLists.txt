if(NOT BUILD_UNITREE)
    return()
endif()

cmake_minimum_required(VERSION 2.8.3)
project(unitree_legged_sdk)
find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        geometry_msgs
)

catkin_package(
        CATKIN_DEPENDS
        roscpp
        std_msgs
        
        INCLUDE_DIRS include
        LIBRARIES ${PROJECT_SOURCE_DIR}/lib/cpp/${ARCH}/libunitree_legged_sdk.a
)

# check arch and os
message("-- CMAKE_SYSTEM_PROCESSOR: ${CMAKE_SYSTEM_PROCESSOR}")
if("${CMAKE_SYSTEM_PROCESSOR}" MATCHES "x86_64.*")
  set(ARCH amd64)
endif()
if("${CMAKE_SYSTEM_PROCESSOR}" MATCHES "aarch64.*")
  set(ARCH arm64)
endif()

include_directories(include ${catkin_INCLUDE_DIRS})
link_directories(lib/cpp/${ARCH})

option(PYTHON_BUILD "build python wrapper" OFF)
if(PYTHON_BUILD)
  add_subdirectory(python_wrapper)
endif()

set(EXTRA_LIBS -pthread libunitree_legged_sdk.a)
set(CMAKE_CXX_FLAGS "-O3 -fPIC")
set(CMAKE_CXX_STANDARD 14)

# set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

# one pc one process
add_executable(unitree_nav unitree_nav/unitree_nav.cpp)
target_link_libraries(unitree_nav ${EXTRA_LIBS} ${catkin_LIBRARIES})

install(
        TARGETS
        unitree_nav
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)