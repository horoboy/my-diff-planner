#!/usr/bin/env bash
set -euo pipefail

workspace="/root/catkin_ws"
compile_database="${workspace}/build/compile_commands.json"

# Keep the locally built NLopt library discoverable after every container start.
echo "${workspace}/nlopt/build" > /etc/ld.so.conf.d/nlopt.conf
ldconfig

# A fresh/rebuilt container may have the source bind-mounted without a compile
# database. Configure catkin once so VS Code gets the exact include paths and
# compiler definitions without forcing a full link during container startup.
if [[ ! -s "${compile_database}" ]]; then
  source /opt/ros/noetic/setup.bash
  cmake -S "${workspace}/src" -B "${workspace}/build" \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
    -DCATKIN_DEVEL_PREFIX="${workspace}/devel" \
    -DCMAKE_INSTALL_PREFIX="${workspace}/install"
fi

# Generated ROS message/config headers do not exist in a completely fresh
# workspace. Build only those lightweight targets when needed.
if [[ ! -f "${workspace}/devel/include/bspline/Bspline.h" ]] ||
   [[ ! -f "${workspace}/devel/include/quadrotor_msgs/PositionCommand.h" ]]; then
  source /opt/ros/noetic/setup.bash
  cmake --build "${workspace}/build" --parallel \
    --target bspline_generate_messages_cpp quadrotor_msgs_generate_messages_cpp
fi
