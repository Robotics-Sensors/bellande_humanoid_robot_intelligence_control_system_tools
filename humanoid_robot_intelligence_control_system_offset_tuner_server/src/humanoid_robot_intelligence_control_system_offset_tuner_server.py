#!/usr/bin/env python3

import rospy
import yaml
from std_msgs.msg import String
from humanoid_robot_intelligence_control_system_offset_tuner_msgs.msg import JointOffsetData, JointTorqueOnOffArray
from humanoid_robot_intelligence_control_system_offset_tuner_msgs.srv import GetPresentJointOffsetData, GetPresentJointOffsetDataResponse
from humanoid_robot_intelligence_control_system_framework import RobotisController, BaseModule
import dynamixel_sdk as dynamixel

OFFSET_ROSPARAM_KEY = "offset"
OFFSET_INIT_POS_ROSPARAM_KEY = "init_pose_for_offset_tuner"

SUB_CONTROLLER_DEVICE = "/dev/ttyUSB0"  # Adjust as needed
BAUD_RATE = 1000000  # Adjust as needed
PROTOCOL_VERSION = 2.0  # Adjust as needed
SUB_CONTROLLER_ID = 200  # Adjust as needed
POWER_CTRL_TABLE = 24  # Adjust as needed

class JointOffsetData:
    def __init__(self, init_pos_rad, offset_rad):
        self.joint_init_pos_rad_ = init_pos_rad
        self.joint_offset_rad_ = offset_rad
        self.p_gain_ = 0
        self.i_gain_ = 0
        self.d_gain_ = 0

class OffsetTunerServer:
    def __init__(self):
        self.offset_file_ = ""
        self.robot_file_ = ""
        self.init_file_ = ""
        self.controller_ = None
        self.robot_offset_data_ = {}
        self.robot_torque_enable_data_ = {}

    def set_ctrl_module(self, module):
        pub = rospy.Publisher('/humanoid_robot_intelligence_control_system/enable_ctrl_module', String, queue_size=1)
        pub.publish(module)

    def initialize(self):
        self.controller_ = RobotisController.getInstance()

        port_handler = dynamixel.PortHandler(SUB_CONTROLLER_DEVICE)
        set_port = port_handler.setBaudRate(BAUD_RATE)
        if not set_port:
            rospy.logerr("Error Set port")
        packet_handler = dynamixel.PacketHandler(PROTOCOL_VERSION)

        return_data = packet_handler.write1ByteTxRx(port_handler, SUB_CONTROLLER_ID, POWER_CTRL_TABLE, 1)

        if return_data != 0:
            rospy.logerr(f"Torque on DXLs! [{packet_handler.getRxPacketError(return_data)}]")
        else:
            rospy.loginfo("Torque on DXLs!")

        port_handler.closePort()

        # Get File Path
        self.offset_file_ = rospy.get_param("offset_path", "")
        self.robot_file_ = rospy.get_param("robot_file_path", "")
        self.init_file_ = rospy.get_param("init_file_path", "")

        if not self.offset_file_ or not self.robot_file_:
            rospy.logerr("Failed to get file path")
            return False

        # Controller Initialize with robot file info
        if not self.controller_.initialize(self.robot_file_, self.init_file_):
            rospy.logerr("ROBOTIS Controller Initialize Fail!")
            return False

        self.controller_.addMotionModule(BaseModule.getInstance())

        # Initialize RobotOffsetData
        for joint_name in self.controller_.robot_.dxls_.keys():
            self.robot_offset_data_[joint_name] = JointOffsetData(0, 0)
            self.robot_torque_enable_data_[joint_name] = True

        # Load Offset.yaml
        self.get_init_pose(self.offset_file_)

        rospy.loginfo(" ")
        rospy.loginfo(" ")
        # For Data Check, Print All Available Data
        rospy.loginfo("num | joint_name | InitPose, OffsetAngleRad")
        for i, (joint_name, joint_data) in enumerate(self.robot_offset_data_.items()):
            rospy.loginfo(f"{i} | {joint_name} : {joint_data.joint_init_pos_rad_}, {joint_data.joint_offset_rad_}")

        rospy.Subscriber("/humanoid_robot_intelligence_control_system/base/send_tra", String, self.string_msgs_callback)
        rospy.Subscriber("/humanoid_robot_intelligence_control_system/offset_tuner/joint_offset_data", JointOffsetData, self.joint_offset_data_callback)
        rospy.Subscriber("/humanoid_robot_intelligence_control_system/offset_tuner/torque_enable", JointTorqueOnOffArray, self.joint_torque_on_off_callback)
        rospy.Subscriber("/humanoid_robot_intelligence_control_system/offset_tuner/command", String, self.command_callback)
        rospy.Service("humanoid_robot_intelligence_control_system/offset_tuner/get_present_joint_offset_data", GetPresentJointOffsetData, self.get_present_joint_offset_data_service_callback)

        return True

    def move_to_init_pose(self):
        self.get_init_pose(self.offset_file_)

        self.controller_.startTimer()
        base_module = BaseModule.getInstance()

        offset_init_pose = {}
        for joint_name, joint_data in self.robot_offset_data_.items():
            if joint_name in self.controller_.robot_.dxls_:
                offset_init_pose[joint_name] = joint_data.joint_init_pos_rad_ + joint_data.joint_offset_rad_

        rospy.sleep(0.08)
        base_module.poseGenerateProc(offset_init_pose)
        rospy.sleep(0.01)

        while base_module.isRunning():
            rospy.sleep(0.05)

        self.controller_.stopTimer()
        while self.controller_.isTimerRunning():
            rospy.sleep(0.01)

        if self.controller_.isTimerRunning():
            rospy.loginfo("Timer Running")

        self.set_ctrl_module("none")

    def string_msgs_callback(self, msg):
        rospy.loginfo(msg.data)

    def command_callback(self, msg):
        if msg.data == "save":
            yaml_out = {}
            offset = {}
            init_pose = {}
            for joint_name, joint_data in self.robot_offset_data_.items():
                offset[joint_name] = joint_data.joint_offset_rad_
                init_pose[joint_name] = joint_data.joint_init_pos_rad_

            yaml_out[OFFSET_ROSPARAM_KEY] = offset
            yaml_out[OFFSET_INIT_POS_ROSPARAM_KEY] = init_pose

            with open(self.offset_file_, 'w') as fout:
                yaml.dump(yaml_out, fout, default_flow_style=False)

        elif msg.data == "ini_pose":
            self.move_to_init_pose()
        else:
            rospy.loginfo(f"Invalid Command : {msg.data}")

    def joint_offset_data_callback(self, msg):
        if self.controller_.isTimerRunning():
            rospy.logerr("Timer is running now")
            return

        rospy.loginfo(f"{msg.joint_name} {msg.goal_value} {msg.offset_value} {msg.p_gain} {msg.i_gain} {msg.d_gain}")

        if msg.joint_name not in self.robot_offset_data_:
            rospy.logerr("Invalid Joint Name")
            return

        if not self.robot_torque_enable_data_[msg.joint_name]:
            rospy.logerr(f"{msg.joint_name} is turned off the torque")
            return

        goal_pose_rad = msg.offset_value + msg.goal_value
        goal_pose_value = self.controller_.robot_.dxls_[msg.joint_name].convertRadian2Value(goal_pose_rad)

        comm_result = self.controller_.writeCtrlItem(
            msg.joint_name,
            self.controller_.robot_.dxls_[msg.joint_name].goal_position_item_.item_name_,
            goal_pose_value
        )

        if comm_result != dynamixel.COMM_SUCCESS:
            rospy.logerr("Failed to write goal position")
            return
        else:
            self.robot_offset_data_[msg.joint_name].joint_init_pos_rad_ = msg.goal_value
            self.robot_offset_data_[msg.joint_name].joint_offset_rad_ = msg.offset_value

        self.robot_offset_data_[msg.joint_name].p_gain_ = msg.p_gain
        self.robot_offset_data_[msg.joint_name].i_gain_ = msg.i_gain
        self.robot_offset_data_[msg.joint_name].d_gain_ = msg.d_gain

    def joint_torque_on_off_callback(self, msg):
        for joint_data in msg.torque_enable_data:
            joint_name = joint_data.joint_name
            torque_enable = joint_data.torque_enable

            if joint_name not in self.robot_offset_data_:
                rospy.logerr("Invalid Joint Name")
                continue

            torque_enable_value = 1 if torque_enable else 0

            comm_result = self.controller_.writeCtrlItem(
                joint_name,
                self.controller_.robot_.dxls_[joint_name].torque_enable_item_.item_name_,
                torque_enable_value
            )

            if comm_result != dynamixel.COMM_SUCCESS:
                rospy.logerr("Failed to write goal position")
            else:
                self.robot_torque_enable_data_[joint_name] = torque_enable

    def get_present_joint_offset_data_service_callback(self, req):
        rospy.loginfo("GetPresentJointOffsetDataService Called")

        res = GetPresentJointOffsetDataResponse()

        for joint_name, joint_data in self.robot_offset_data_.items():
            if self.controller_.robot_.dxls_[joint_name].present_position_item_ is None:
                continue

            present_pos_value = self.controller_.readCtrlItem(
                joint_name,
                self.controller_.robot_.dxls_[joint_name].present_position_item_.item_name_
            )

            if present_pos_value is None:
                rospy.logerr("Failed to read present pos")
                return False

            joint_offset_pos = JointOffsetData()
            joint_offset_pos.joint_name = joint_name
            joint_offset_pos.goal_value = joint_data.joint_init_pos_rad_
            joint_offset_pos.offset_value = joint_data.joint_offset_rad_
            joint_offset_pos.present_value = self.controller_.robot_.dxls_[joint_name].convertValue2Radian(present_pos_value)
            joint_offset_pos.p_gain = joint_data.p_gain_
            joint_offset_pos.i_gain = joint_data.i_gain_
            joint_offset_pos.d_gain = joint_data.d_gain_

            res.present_data_array.append(joint_offset_pos)

        return res

    def get_init_pose(self, path):
        try:
            with open(path, 'r') as f:
                offset_yaml = yaml.safe_load(f)
        except Exception as e:
            rospy.logerr("Fail to load offset yaml file.")
            offset_yaml = None

        if offset_yaml:
            offset_data = offset_yaml.get(OFFSET_ROSPARAM_KEY, {})
            for joint_name, offset in offset_data.items():
                if joint_name in self.robot_offset_data_:
                    self.robot_offset_data_[joint_name].joint_offset_rad_ = offset

            offset_init_pose = offset_yaml.get(OFFSET_INIT_POS_ROSPARAM_KEY, {})
            for joint_name, offset_init_pose in offset_init_pose.items():
                if joint_name in self.robot_offset_data_:
                    self.robot_offset_data_[joint_name].joint_init_pos_rad_ = offset_init_pose
        else:
            default_offset_value = 0.0
            for joint_name in self.controller_.robot_.dxls_.keys():
                self.robot_offset_data_[joint_name].joint_offset_rad_ = default_offset_value
                self.robot_offset_data_[joint_name].joint_init_pos_rad_ = default_offset_value

if __name__ == '__main__':
    rospy.init_node('offset_tuner_server')
    offset_tuner_server = OffsetTunerServer()
    if offset_tuner_server.initialize():
        rospy.spin()
