#SPDX-License-Identifier: Apache-2.0
cmake_minimum_required(VERSION 3.0.2)
project(mikrik_teleop)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  joy
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(mikrik_teleop src/mikrik_teleop.cpp)
target_link_libraries(mikrik_teleop ${catkin_LIBRARIES})
