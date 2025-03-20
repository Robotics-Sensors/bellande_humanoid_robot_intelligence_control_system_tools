# Copyright (C) 2024 Bellande Robotics Sensors Research Innovation Center, Ronaldson Bellande
# 
# Licensed under the Apache License, Version 2.0 (the "License"); you may not
# use this file except in compliance with the License. You may obtain a copy of
# the License at
# 
# http://www.apache.org/licenses/LICENSE-2.0
# 
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
# License for the specific language governing permissions and limitations under
# the License.

cmake_minimum_required(VERSION 3.8)
project(humanoid_robot_intelligence_control_system_action_editor)
## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

# Find ROS
if($ENV{ROS_VERSION} EQUAL 1)
  find_package(
    catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    dynamixel_sdk
    humanoid_robot_intelligence_control_system_controller
    humanoid_robot_intelligence_control_system_device
    humanoid_robot_intelligence_control_system_action_module
    humanoid_robot_intelligence_control_system_base_module
  )

  find_package(Curses REQUIRED)
  find_package(PkgConfig REQUIRED)
else()
  find_package(ament_cmake REQUIRED)
endif()


pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)
find_path(
  YAML_CPP_INCLUDE_DIR
  NAMES yaml_cpp.h
  PATHS ${YAML_CPP_INCLUDE_DIRS}
)
find_library(
  YAML_CPP_LIBRARY
  NAMES YAML_CPP
  PATHS ${YAML_CPP_LIBRARY_DIRS}
)
link_directories(${YAML_CPP_LIBRARY_DIRS})

if(NOT ${YAML_CPP_VERSION} VERSION_LESS "0.5")
  add_definitions(-DHAVE_NEW_YAMLCPP)
endif(NOT ${YAML_CPP_VERSION} VERSION_LESS "0.5")

if($ENV{ROS_VERSION} EQUAL 1)
  catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS
    roscpp
    std_msgs
    dynamixel_sdk
    humanoid_robot_intelligence_control_system_controller
    humanoid_robot_intelligence_control_system_device
    humanoid_robot_intelligence_control_system_action_module
    humanoid_robot_intelligence_control_system_base_module
    DEPENDS CURSES
  )
endif()


################################################################################
# Build
################################################################################
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
  ${CURSES_INCLUDE_DIRS}
)

