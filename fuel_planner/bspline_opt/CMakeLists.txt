cmake_minimum_required(VERSION 2.8.3)
project(bspline_opt)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

get_filename_component(WORKSPACE_ROOT "${CMAKE_SOURCE_DIR}/.." ABSOLUTE)
set(NLOPT_ROOT "${WORKSPACE_ROOT}/nlopt" CACHE PATH "Path to NLopt source/build tree")

find_path(NLOPT_INCLUDE_DIR
  NAMES nlopt.h
  HINTS
    "${NLOPT_ROOT}/src/api"
    /usr/local/include
    /usr/include
)

find_path(NLOPT_CXX_INCLUDE_DIR
  NAMES nlopt.hpp
  HINTS
    "${NLOPT_ROOT}/build"
    /usr/local/include
    /usr/include
)

find_library(NLOPT_LIBRARY
  NAMES nlopt
  HINTS
    "${NLOPT_ROOT}/build"
    /usr/local/lib
    /usr/lib
    /usr/lib/x86_64-linux-gnu
)

if(NOT NLOPT_INCLUDE_DIR OR NOT NLOPT_CXX_INCLUDE_DIR OR NOT NLOPT_LIBRARY)
  message(FATAL_ERROR "NLopt was not found. Set NLOPT_ROOT to the NLopt source/build tree or install libnlopt-dev.")
endif()

set(NLOPT_INCLUDE_DIRS ${NLOPT_INCLUDE_DIR} ${NLOPT_CXX_INCLUDE_DIR})
message(STATUS "Using NLopt library: ${NLOPT_LIBRARY}")
message(STATUS "Using NLopt includes: ${NLOPT_INCLUDE_DIRS}")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  visualization_msgs
  plan_env
  active_perception
  cv_bridge
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES bspline_opt
 CATKIN_DEPENDS plan_env active_perception
)


include_directories( 
    SYSTEM 
    include 
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
    ${NLOPT_INCLUDE_DIRS}
)

add_library( bspline_opt 
    src/bspline_optimizer.cpp 
    )
target_link_libraries( bspline_opt
    ${catkin_LIBRARIES} 
    ${NLOPT_LIBRARY}   
    )
