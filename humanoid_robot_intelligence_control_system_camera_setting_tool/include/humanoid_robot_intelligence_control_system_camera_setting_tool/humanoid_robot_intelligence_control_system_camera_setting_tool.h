/*******************************************************************************
 * Copyright 2017 ROBOTIS CO., LTD.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *******************************************************************************/

/* Author: Kayman Jung */

#ifndef HUMANOID_ROBOT_CAMERA_SETTING_TOOL_H_
#define HUMANOID_ROBOT_CAMERA_SETTING_TOOL_H_

#include <boost/thread.hpp>
#include <dynamic_reconfigure/server.h>
#include <fstream>
#include <ros/package.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <yaml-cpp/yaml.h>

#include "humanoid_robot_intelligence_control_system_camera_setting_tool/CameraParamsConfig.h"
#include "humanoid_robot_intelligence_control_system_camera_setting_tool/V4lParameter.h"
#include "humanoid_robot_intelligence_control_system_camera_setting_tool/V4lParameters.h"

#include "humanoid_robot_intelligence_control_system_camera_setting_tool/GetParameters.h"
#include "humanoid_robot_intelligence_control_system_camera_setting_tool/SetParameters.h"

std::string g_device_name;
std::string g_camera_node_name;
std::map<std::string, std::string> g_param_list;

boost::shared_ptr<
    dynamic_reconfigure::Server<humanoid_robot_intelligence_control_system_camera_setting_tool::CameraParamsConfig>>
    g_param_server;
humanoid_robot_intelligence_control_system_camera_setting_tool::CameraParamsConfig g_dyn_config;

// web setting
std::string g_default_setting_path;
ros::Publisher g_param_pub;
ros::Subscriber g_param_command_sub;
ros::ServiceServer g_get_param_client;
ros::ServiceServer g_set_param_client;

bool g_has_path;
std::string g_param_path;

void dynParamCallback(humanoid_robot_intelligence_control_system_camera_setting_tool::CameraParamsConfig &config,
                      uint32_t level);
void changeDynParam(const std::string &param, const int &value);
void updateDynParam(humanoid_robot_intelligence_control_system_camera_setting_tool::CameraParamsConfig &config);

void setCameraParameterCallback(
    const humanoid_robot_intelligence_control_system_camera_setting_tool::V4lParameter::ConstPtr &msg);
void setCameraParametersCallback(
    const humanoid_robot_intelligence_control_system_camera_setting_tool::V4lParameters::ConstPtr &msg);

void setV4lParameter(const std::string &param, const std::string &value);
void setV4lParameter(const std::string &param, const int &value);
void setV4lParameter(const std::string &cmd);

void getROSParam();
void setROSParam(const std::string &param, const int &value);

void paramCommandCallback(const std_msgs::String::ConstPtr &msg);
bool setParamCallback(humanoid_robot_intelligence_control_system_camera_setting_tool::SetParameters::Request &req,
                      humanoid_robot_intelligence_control_system_camera_setting_tool::SetParameters::Response &res);
bool getParamCallback(humanoid_robot_intelligence_control_system_camera_setting_tool::GetParameters::Request &req,
                      humanoid_robot_intelligence_control_system_camera_setting_tool::GetParameters::Response &res);
void resetParameter();
void saveParameter();
void publishParam();

#endif /* HUMANOID_ROBOT_CAMERA_SETTING_TOOL_H_ */
