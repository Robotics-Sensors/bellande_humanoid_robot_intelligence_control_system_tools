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
project(humanoid_robot_intelligence_control_system_camera_setting_tool)

# Find ROS
if($ENV{ROS_VERSION} EQUAL 1)
  find_package(
    catkin REQUIRED COMPONENTS
    roscpp
    roslib
    std_msgs
    dynamic_reconfigure
    message_generation
  )
  
  find_package(Boost REQUIRED COMPONENTS thread)
  find_package(PkgConfig REQUIRED)
  
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
  endif()
  
  add_message_files(
    FILES
    V4lParameter.msg
    V4lParameters.msg
    CameraParams.msg
  )
  
  add_service_files(
    FILES
    GetParameters.srv
    SetParameters.srv
  )
  
  generate_messages(
    DEPENDENCIES
    std_msgs
  )
  
  generate_dynamic_reconfigure_options(cfg/CameraParams.cfg)
  
  catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS
    roscpp
    roslib
    std_msgs
    dynamic_reconfigure
    message_runtime
    DEPENDS Boost
  )
  
else()
  find_package(ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(rosidl_default_generators REQUIRED)
  
  find_package(Boost REQUIRED COMPONENTS thread)
  find_package(PkgConfig REQUIRED)
  
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
  endif()
  
  rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/V4lParameter.msg"
    "msg/V4lParameters.msg"
    "msg/CameraParams.msg"
    "srv/GetParameters.srv"
    "srv/SetParameters.srv"
    DEPENDENCIES std_msgs
  )
  
  ament_export_dependencies(rosidl_default_runtime)
  
  ament_package()
endif()
