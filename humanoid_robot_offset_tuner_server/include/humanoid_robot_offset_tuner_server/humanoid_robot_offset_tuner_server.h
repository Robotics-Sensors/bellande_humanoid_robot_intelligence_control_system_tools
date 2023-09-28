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

/* Author: Jay Song */

#ifndef HUMANOID_ROBOT_OFFSET_TUNER_SERVER_H_
#define HUMANOID_ROBOT_OFFSET_TUNER_SERVER_H_

#include <fstream>
#include <map>
#include <ros/ros.h>
#include <yaml-cpp/yaml.h>

#include "dynamixel_sdk/dynamixel_sdk.h"
#include "humanoid_robot_base_module/base_module.h"
#include "humanoid_robot_offset_tuner_msgs/GetPresentJointOffsetData.h"
#include "humanoid_robot_offset_tuner_msgs/JointOffsetData.h"
#include "humanoid_robot_offset_tuner_msgs/JointTorqueOnOffArray.h"
#include "humanoid_robot_controller/humanoid_robot_controller.h"

namespace humanoid_robot_op {

class JointOffsetData {
public:
  double joint_offset_rad_;
  double joint_init_pos_rad_;
  int p_gain_;
  int i_gain_;
  int d_gain_;

  JointOffsetData() {
    joint_offset_rad_ = 0;
    joint_init_pos_rad_ = 0;
    p_gain_ = 800;
    i_gain_ = 0;
    d_gain_ = 0;
  }

  JointOffsetData(double joint_offset_rad, double joint_init_pose_rad) {
    this->joint_offset_rad_ = joint_offset_rad;
    this->joint_init_pos_rad_ = joint_init_pose_rad;
    p_gain_ = 800;
    i_gain_ = 0;
    d_gain_ = 0;
  }

  ~JointOffsetData() {}
};

class OffsetTunerServer
    : public humanoid_robot_framework::Singleton<OffsetTunerServer> {

public:
  OffsetTunerServer();
  ~OffsetTunerServer();

  bool initialize();
  void moveToInitPose();
  void stringMsgsCallBack(const std_msgs::String::ConstPtr &msg);
  void commandCallback(const std_msgs::String::ConstPtr &msg);
  void jointOffsetDataCallback(
      const humanoid_robot_offset_tuner_msgs::JointOffsetData::ConstPtr &msg);
  void jointTorqueOnOffCallback(
      const humanoid_robot_offset_tuner_msgs::JointTorqueOnOffArray::ConstPtr &msg);
  bool getPresentJointOffsetDataServiceCallback(
      humanoid_robot_offset_tuner_msgs::GetPresentJointOffsetData::Request &req,
      humanoid_robot_offset_tuner_msgs::GetPresentJointOffsetData::Response &res);

private:
  const int BAUD_RATE = 2000000;
  const double PROTOCOL_VERSION = 2.0;
  const int SUB_CONTROLLER_ID = 200;
  const char *SUB_CONTROLLER_DEVICE = "/dev/ttyUSB0";
  const int POWER_CTRL_TABLE = 24;

  void setCtrlModule(std::string module);
  void getInitPose(const std::string &path);

  humanoid_robot_framework::RobotisController *controller_;

  std::string offset_file_;
  std::string robot_file_;
  std::string init_file_;
  std::map<std::string, JointOffsetData *> robot_offset_data_;
  std::map<std::string, bool> robot_torque_enable_data_;

  ros::Subscriber send_tra_sub_;
  ros::Subscriber joint_offset_data_sub_;
  ros::Subscriber joint_torque_enable_sub_;
  ros::Subscriber command_sub_;
  ros::ServiceServer offset_data_server_;
};

} // namespace humanoid_robot_op

#endif /* HUMANOID_ROBOT_OFFSET_TUNER_SERVER_H_ */
