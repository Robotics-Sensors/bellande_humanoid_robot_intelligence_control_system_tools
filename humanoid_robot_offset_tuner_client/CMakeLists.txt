cmake_minimum_required(VERSION 3.8)
project(humanoid_robot_offset_tuner_client)

if($ENV{ROS_VERSION} EQUAL 1)
  find_package(
    catkin REQUIRED COMPONENTS
    rqt_gui
    rqt_gui_cpp
    roscpp
    roslib
    humanoid_robot_offset_tuner_msgs
  )

  find_package(Qt5Widgets REQUIRED)
else()
  find_package(ament_cmake REQUIRED)
endif()



set(
  humanoid_robot_offset_tuner_client_SRCS
  src/main.cpp
  src/main_window.cpp
  src/qnode.cpp
)

set(
  humanoid_robot_offset_tuner_client_HDRS
  include/humanoid_robot_offset_tuner_client/main_window.hpp
  include/humanoid_robot_offset_tuner_client/qnode.hpp
)

set(
  humanoid_robot_offset_tuner_client_UIS
  ui/main_window.ui
)

set(
  humanoid_robot_offset_tuner_client_RESOURCES
  resources/images.qrc
)

set(
  humanoid_robot_offset_tuner_client_INCLUDE_DIRECTORIES
  include
  include/${PROJECT_NAME}
  "${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}/${PROJECT_NAME}"
)
if(NOT EXISTS "${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}/${PROJECT_NAME}")
  file(MAKE_DIRECTORY "${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}/${PROJECT_NAME}")
endif()

if($ENV{ROS_VERSION} EQUAL 1)
  catkin_package(
    INCLUDE_DIRS 
    ${humanoid_robot_offset_tuner_client_INCLUDE_DIRECTORIES}
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS 
    rqt_gui 
    rqt_gui_cpp 
    roscpp 
    roslib 
    humanoid_robot_offset_tuner_msgs
  )
else()
  ament_package()
endif()


qt5_wrap_cpp(humanoid_robot_offset_tuner_client_MOCS ${humanoid_robot_offset_tuner_client_HDRS})

# ensure generated header files are being created in the devel space
set(_cmake_current_binary_dir "${CMAKE_CURRENT_BINARY_DIR}")
set(CMAKE_CURRENT_BINARY_DIR "${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}/${PROJECT_NAME}")
if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  qt5_wrap_ui(humanoid_robot_offset_tuner_client_UIS_H ${humanoid_robot_offset_tuner_client_UIS})
else()
  qt4_wrap_ui(humanoid_robot_offset_tuner_client_UIS_H ${humanoid_robot_offset_tuner_client_UIS})
endif()
set(CMAKE_CURRENT_BINARY_DIR "${_cmake_current_binary_dir}")

if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  qt5_add_resources(humanoid_robot_offset_tuner_client_RCC ${humanoid_robot_offset_tuner_client_RESOURCES})
else()
  qt4_add_resources(humanoid_robot_offset_tuner_client_RCC ${humanoid_robot_offset_tuner_client_RESOURCES})
endif()


include_directories(
  ${humanoid_robot_offset_tuner_client_INCLUDE_DIRECTORIES}
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)
add_executable(${PROJECT_NAME} ${humanoid_robot_offset_tuner_client_SRCS} ${humanoid_robot_offset_tuner_client_MOCS} ${humanoid_robot_offset_tuner_client_UIS_H} ${humanoid_robot_offset_tuner_client_RCC})
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} Qt5::Widgets yaml-cpp)

find_package(class_loader)
class_loader_hide_library_symbols(${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME} RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
