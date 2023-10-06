cmake_minimum_required(VERSION 3.8)
project(humanoid_robot_action_editor)
## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

# Find ROS
if($ENV{ROS_VERSION} EQUAL 1)
  find_package(
    catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    dynamixel_sdk
    humanoid_robot_controller
    humanoid_robot_device
    humanoid_robot_action_module
    humanoid_robot_base_module
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
    humanoid_robot_controller
    humanoid_robot_device
    humanoid_robot_action_module
    humanoid_robot_base_module
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

add_executable(${PROJECT_NAME} src/main.cpp src/action_editor.cpp)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES} ${CURSES_LIBRARIES})


################################################################################
# Install
################################################################################
install(
  TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(
  DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

################################################################################
# Test
################################################################################
