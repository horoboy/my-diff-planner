cmake_minimum_required(VERSION 3.0.2)
project(fuel_command_safety)

add_compile_options(-std=c++14 -Wall -Wextra)

find_package(catkin REQUIRED COMPONENTS
  diagnostic_msgs
  nav_msgs
  quadrotor_msgs
  roscpp
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES command_safety
  CATKIN_DEPENDS diagnostic_msgs nav_msgs quadrotor_msgs roscpp std_msgs
)

include_directories(
  include
  SYSTEM
  ${catkin_INCLUDE_DIRS}
)

add_library(command_safety src/command_safety.cpp)
target_link_libraries(command_safety ${catkin_LIBRARIES})
add_dependencies(command_safety ${catkin_EXPORTED_TARGETS})

add_executable(command_safety_node src/command_safety_node.cpp)
target_link_libraries(command_safety_node command_safety ${catkin_LIBRARIES})
add_dependencies(command_safety_node ${catkin_EXPORTED_TARGETS})

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(command_safety_test test/command_safety_test.cpp)
  if(TARGET command_safety_test)
    target_link_libraries(command_safety_test command_safety ${catkin_LIBRARIES})
  endif()
endif()

install(TARGETS command_safety command_safety_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
