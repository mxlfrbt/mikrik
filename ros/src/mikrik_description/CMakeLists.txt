#SPDX-License-Identifier: Apache-2.0
cmake_minimum_required(VERSION 3.0.2)
project(mikrik_description)

find_package(catkin REQUIRED COMPONENTS
  tf
  urdf
  xacro
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
