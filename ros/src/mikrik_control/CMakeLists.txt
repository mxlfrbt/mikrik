#SPDX-License-Identifier: Apache-2.0
cmake_minimum_required(VERSION 2.8.3)
project(mikrik_control)

find_package(catkin REQUIRED COMPONENTS
  controller_manager
  hardware_interface
  trajectory_msgs
)

catkin_package(
  CATKIN_DEPENDS
    controller_manager
    hardware_interface
    trajectory_msgs
)
