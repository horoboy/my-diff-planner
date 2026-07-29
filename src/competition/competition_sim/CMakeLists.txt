cmake_minimum_required(VERSION 3.0.2)
project(competition_sim)

find_package(catkin REQUIRED COMPONENTS
  competition_msgs
  geometry_msgs
  nav_msgs
  quadrotor_msgs
  rospy
  sensor_msgs
  std_msgs
  std_srvs
  visualization_msgs
)

catkin_package()

catkin_install_python(
  PROGRAMS
    scripts/competition_forest_map.py
    scripts/competition_mission_manager.py
    scripts/command_mux.py
    scripts/cloud_restamper.py
    scripts/coverage_search_server.py
    scripts/mission_evaluator.py
    scripts/payload_sim.py
    scripts/planner_watchdog.py
    scripts/run_competition_batch.py
    scripts/run_single_uav_validation.sh
    scripts/target_detector_sim.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
