cmake_minimum_required(VERSION 3.0.2)
project(fuel_realflight_adapter)

add_compile_options(-std=c++14 -Wall -Wextra)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  sensor_msgs
)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp sensor_msgs
  DEPENDS EIGEN3
)

include_directories(
  include
  SYSTEM
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(cloud_pose_adapter_node src/cloud_pose_adapter_node.cpp)
target_link_libraries(cloud_pose_adapter_node ${catkin_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  catkin_add_gtest(pose_math_test test/pose_math_test.cpp)
  if(TARGET pose_math_test)
    target_link_libraries(pose_math_test ${catkin_LIBRARIES})
  endif()

  add_rostest_gtest(cloud_pose_adapter_integration_test
    test/cloud_pose_adapter.test
    test/cloud_pose_adapter_integration_test.cpp
  )
  if(TARGET cloud_pose_adapter_integration_test)
    target_link_libraries(cloud_pose_adapter_integration_test ${catkin_LIBRARIES})
  endif()
endif()

install(TARGETS cloud_pose_adapter_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
