cmake_minimum_required(VERSION 3.8)
project(humanoid_robot_offset_tuner_server)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

if($ENV{ROS_VERSION} EQUAL 1)
  find_package(
    catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    humanoid_robot_offset_tuner_msgs
    dynamixel_sdk
    humanoid_robot_controller
    humanoid_robot_base_module
  )

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
    humanoid_robot_offset_tuner_msgs
    dynamixel_sdk
    humanoid_robot_controller
    humanoid_robot_base_module
  )
endif()


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
)

add_executable(offset_tuner_server_node src/humanoid_robot_offset_tuner_server.cpp src/main.cpp)
add_dependencies(offset_tuner_server_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(offset_tuner_server_node ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})

install(
  TARGETS offset_tuner_server_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(
  DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

