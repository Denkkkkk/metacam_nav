cmake_minimum_required(VERSION 3.5)
project(bbs3d)

## find thirdparty/Eigen or Eigen3 package
option(USE_THIRDPARTY_EIGEN "Include thirdparty eigen" ON)
file(GLOB entries "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/Eigen/*")
list(LENGTH entries num_entries)
message(STATUS "num_entries in thirdparty/Eigen: ${num_entries}")
if(num_entries EQUAL 0)
  find_package(Eigen3 REQUIRED)
else()
  set(USE_THIRDPARTY_EIGEN ON)
endif()

## find OpenMP
find_package(OpenMP)
if (OPENMP_FOUND)
  set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

set(EIGEN3_FOUND FALSE)
unset(EIGEN3_INCLUDE_DIR CACHE)
# Common include directories
if(USE_THIRDPARTY_EIGEN)
  set(THIRDPARTY_EIGEN_DIR ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/Eigen)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -I${THIRDPARTY_EIGEN_DIR}")
  include_directories(${THIRDPARTY_EIGEN_DIR})
  message(STATUS "Use thirdparty Eigen from: ${THIRDPARTY_EIGEN_DIR}")
else()
  include_directories(${EIGEN3_INCLUDE_DIR})
  message(STATUS "Use EIGEN3_INCLUDE_DIR")
endif()

## find CUDA
if (BUILD_CUDA)
  find_package(CUDA REQUIRED)
endif()

if(BUILD_CUDA)
  cuda_add_library(gpu_bbs3d
    SHARED 
    bbs3d/src/gpu_bbs3d/bbs3d.cu
    bbs3d/src/gpu_bbs3d/calc_score.cu
    bbs3d/src/gpu_bbs3d/voxelmaps.cu
    bbs3d/src/gpu_bbs3d/stream_manager/check_error.cu
    bbs3d/src/gpu_bbs3d/voxelmaps_io.cu
  )
  target_include_directories(gpu_bbs3d PUBLIC ${CUDA_INCLUDE_DIRS} bbs3d/include)
  target_link_libraries(gpu_bbs3d ${CUDA_LIBRARIES})
  install(
    TARGETS
    gpu_bbs3d
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
else()
  add_library(cpu_bbs3d
  SHARED
  bbs3d/src/cpu_bbs3d/bbs3d.cpp
  bbs3d/src/cpu_bbs3d/voxelmaps.cpp
  bbs3d/src/cpu_bbs3d/voxelmaps_io.cpp)
  target_include_directories(cpu_bbs3d PUBLIC  bbs3d/include)
  install(
    TARGETS
    cpu_bbs3d
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
endif()

