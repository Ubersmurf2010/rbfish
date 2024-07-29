# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/home/ubersmurf/ros2_ws/build/micro_ros_agent/agent/src/xrceagent"
  "/home/ubersmurf/ros2_ws/build/micro_ros_agent/agent/src/xrceagent-build"
  "/home/ubersmurf/ros2_ws/build/micro_ros_agent/agent/src/xrceagent-build/uagent-prefix"
  "/home/ubersmurf/ros2_ws/build/micro_ros_agent/agent/src/xrceagent-build/uagent-prefix/tmp"
  "/home/ubersmurf/ros2_ws/build/micro_ros_agent/agent/src/xrceagent-build/uagent-prefix/src/uagent-stamp"
  "/home/ubersmurf/ros2_ws/build/micro_ros_agent/agent/src/xrceagent-build/uagent-prefix/src"
  "/home/ubersmurf/ros2_ws/build/micro_ros_agent/agent/src/xrceagent-build/uagent-prefix/src/uagent-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/ubersmurf/ros2_ws/build/micro_ros_agent/agent/src/xrceagent-build/uagent-prefix/src/uagent-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/ubersmurf/ros2_ws/build/micro_ros_agent/agent/src/xrceagent-build/uagent-prefix/src/uagent-stamp${cfgdir}") # cfgdir has leading slash
endif()
