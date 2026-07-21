cmake_minimum_required(VERSION 3.0.2)
project(fuel_px4ctrl_sitl)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS
  scripts/configure_sitl_streams.py
  scripts/command_fault_injector.py
  scripts/gazebo_ground_truth_odom.py
  scripts/prepare_sitl_takeoff.py
  scripts/run_command_safety_scenarios.py
  scripts/run_command_safety_validation.sh
  scripts/run_isolated_px4ctrl.sh
  scripts/run_isolated_sitl_stack.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
