// ********************************************************************************************************************
// Copyright [2025] Renesas Electronics Corporation and/or its licensors. All Rights Reserved.
//
// The contents of this file (the "contents") are proprietary and confidential to Renesas Electronics Corporation
// and/or its licensors ("Renesas") and subject to statutory and contractual protections.
//
// Unless otherwise expressly agreed in writing between Renesas and you: 1) you may not use, copy, modify, distribute,
// display, or perform the contents; 2) you may not use any name or mark of Renesas for advertising or publicity
// purposes or in connection with your use of the contents; 3) RENESAS MAKES NO WARRANTY OR REPRESENTATIONS ABOUT THE
// SUITABILITY OF THE CONTENTS FOR ANY PURPOSE; THE CONTENTS ARE PROVIDED "AS IS" WITHOUT ANY EXPRESS OR IMPLIED
// WARRANTY, INCLUDING THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, AND
// NON-INFRINGEMENT; AND 4) RENESAS SHALL NOT BE LIABLE FOR ANY DIRECT, INDIRECT, SPECIAL, OR CONSEQUENTIAL DAMAGES,
// INCLUDING DAMAGES RESULTING FROM LOSS OF USE, DATA, OR PROJECTS, WHETHER IN AN ACTION OF CONTRACT OR TORT, ARISING
// OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THE CONTENTS. Third-party contents included in this file may
// be subject to different terms.
// ********************************************************************************************************************
#include "agilex_piper_controller/piper_controller.hpp"

#include <Eigen/Dense>
#include <chrono>
#include <cstring>
#include <iostream>
#include <thread>

namespace agilex
{
namespace piper
{

PiperController::PiperController(
  const std::string & can_interface_name, bool auto_init, int dh_is_offset,
  bool enable_sdk_joint_limits, bool enable_sdk_gripper_limits)
: can_interface_name_(can_interface_name),
  auto_init_(auto_init),
  dh_is_offset_(dh_is_offset),
  enable_sdk_joint_limits_(enable_sdk_joint_limits),
  enable_sdk_gripper_limits_(enable_sdk_gripper_limits),
  running_(false)
{
  // Initialize parameters
  parameters_ = std::make_unique<PiperParams>();

  // Initialize protocol (using V2 protocol)
  protocol_ = std::unique_ptr<PiperProtocolBase>(new PiperProtocolV2());

  // Define the CAN frame callback
  auto callback = [this](CanFrameMsg & frame) { this->parse_can_frame(frame); };

  // Initialize CAN interface
  can_interface_ = std::make_unique<CanInterface>(can_interface_name_, 1000000, callback);

  // Initialize firmware data buffer
  firmware_data_.clear();

  // Initialize the controller if auto_init is true
  if (auto_init_) {
    connect_port();
  }
}

PiperController::~PiperController()
{
  // Stop all threads and cleanup
  disconnect();
}

bool PiperController::connect_port(bool init_can)
{
  if (running_) {
    // Already connected
    return true;
  }

  // Initialize CAN interface if needed
  if (init_can) {
    if (!can_interface_->initialize()) {
      std::cerr << "Failed to initialize CAN interface" << std::endl;
      return false;
    }
  }

  // Start the CAN interface
  if (!can_interface_->start()) {
    std::cerr << "Failed to start CAN interface" << std::endl;
    return false;
  }

  // Start monitoring threads
  running_ = true;
  read_can_thread_ = std::thread(&PiperController::read_can_loop, this);
  can_monitor_thread_ = std::thread(&PiperController::can_monitor_loop, this);

  return true;
}

void PiperController::disconnect()
{
  // Stop monitoring threads
  running_ = false;

  if (read_can_thread_.joinable()) {
    read_can_thread_.join();
  }

  if (can_monitor_thread_.joinable()) {
    can_monitor_thread_.join();
  }

  // Stop CAN interface
  if (can_interface_) {
    can_interface_->stop();
  }
}

bool PiperController::is_connected() const
{
  return running_ && can_interface_ && can_interface_->is_running();
}

ArmStatus PiperController::get_arm_status() const
{
  std::lock_guard<std::mutex> lock(arm_status_mutex_);
  return arm_status_;
}

ArmEndPose PiperController::get_arm_end_pose() const
{
  std::lock_guard<std::mutex> lock(arm_end_pose_mutex_);
  return arm_end_pose_;
}

ArmJoint PiperController::get_arm_joint() const
{
  std::lock_guard<std::mutex> lock(arm_joint_mutex_);
  return arm_joint_;
}

ArmGripper PiperController::get_arm_gripper() const
{
  std::lock_guard<std::mutex> lock(arm_gripper_mutex_);
  return arm_gripper_;
}

bool PiperController::enable_arm()
{
  if (!is_connected()) {
    return false;
  }

  // Reset if current control mode is not 0x01 (CAN instruction control)
  if ((arm_status_.ctrl_mode != 0x01) && (arm_status_.ctrl_mode != 0x00)) {
    motion_control_1(1);
    motion_control_1(2);
    motion_control_2(0, 0, 0, 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }

  // Create a message to enable all joints
  MsgEnableDisableArm msg_data;
  msg_data.motor_num = 0x07;  // Enable all joints
  msg_data.enable_flag = 0x02;
  PiperMessage msg(MessageType::ENABLE_DISABLE_ARM, msg_data);
  CanFrameMsg can_frame;
  if (!protocol_->encode_message(msg, can_frame)) {
    return false;
  }

  // Check if all joints are enabled and try to enable them if not
  const uint8_t JOINT_ENABLED_BIT = 0x40;  // bit[6]: Drive enable status (1: Enabled, 0: Disabled)
  const int MAX_ATTEMPTS = 20;
  const int ATTEMPT_INTERVAL_MS = 10;

  // Make first attempt to enable
  can_interface_->send_message(can_frame);

  // Additional attempts if needed
  for (int attempt = 1; attempt < MAX_ATTEMPTS; ++attempt) {
    // Wait for status to update
    std::this_thread::sleep_for(std::chrono::milliseconds(ATTEMPT_INTERVAL_MS));

    // Check if all joints are enabled
    bool all_joints_enabled = true;
    for (uint8_t joint = 1; joint <= 6; ++joint) {
      MotorInfo info = get_motor_info(joint);
      if ((info.drive_status & JOINT_ENABLED_BIT) == 0) {
        all_joints_enabled = false;
        break;
      }
    }

    // If all joints are enabled, return success
    if (all_joints_enabled) {
      return true;
    }

    // Not all joints are enabled, try again
    can_interface_->send_message(can_frame);
  }

  // Check one last time if all joints are enabled
  bool final_check = true;
  for (uint8_t joint = 1; joint <= 6; ++joint) {
    MotorInfo info = get_motor_info(joint);
    if ((info.drive_status & JOINT_ENABLED_BIT) == 0) {
      std::cerr << "Joint " << static_cast<int>(joint) << " is not enabled after " << MAX_ATTEMPTS
                << " attempts" << std::endl;
      final_check = false;
    }
  }

  return final_check;
}

bool PiperController::disable_arm()
{
  if (!is_connected()) {
    return false;
  }

  MsgEnableDisableArm msg_data;
  msg_data.motor_num = 0x07;  // Disable all joints
  msg_data.enable_flag = 0x01;

  PiperMessage msg(MessageType::ENABLE_DISABLE_ARM, msg_data);
  CanFrameMsg can_frame;

  if (protocol_->encode_message(msg, can_frame)) {
    return can_interface_->send_message(can_frame);
  }

  return false;
}

bool PiperController::set_mode(
  uint8_t ctrl_mode, uint8_t move_mode, uint8_t move_speed_rate, uint8_t is_mit_mode)
{
  if (!is_connected()) {
    return false;
  }

  return motion_control_2(ctrl_mode, move_mode, move_speed_rate, is_mit_mode);
}

bool PiperController::set_end_pose(int x, int y, int z, int rx, int ry, int rz)
{
  if (!is_connected()) {
    return false;
  }

  // Send the pose in three separate messages (XY, ZRX, RYRZ)
  const bool ok_xy = cartesian_ctrl_xy(x, y);
  const bool ok_zrx = cartesian_ctrl_zrx(z, rx);
  const bool ok_ryrz = cartesian_ctrl_ryrz(ry, rz);

  return ok_xy && ok_zrx && ok_ryrz;
}

bool PiperController::set_joint_angles(int j1, int j2, int j3, int j4, int j5, int j6)
{
  if (!is_connected()) {
    return false;
  }

  // Apply SDK joint limits if enabled
  if (enable_sdk_joint_limits_) {
    j1 = check_joint_sdk_limit(j1, "joint1");
    j2 = check_joint_sdk_limit(j2, "joint2");
    j3 = check_joint_sdk_limit(j3, "joint3");
    j4 = check_joint_sdk_limit(j4, "joint4");
    j5 = check_joint_sdk_limit(j5, "joint5");
    j6 = check_joint_sdk_limit(j6, "joint6");
  }

  // Send joint commands in three separate messages (J12, J34, J56)
  const bool ok_12 = joint_ctrl_12(j1, j2);
  const bool ok_34 = joint_ctrl_34(j3, j4);
  const bool ok_56 = joint_ctrl_56(j5, j6);

  return ok_12 && ok_34 && ok_56;
}

bool PiperController::control_gripper(
  int grippers_angle, uint16_t grippers_effort, uint8_t status_code, uint8_t set_zero)
{
  if (!is_connected()) {
    return false;
  }

  MsgGripperCtrl msg_data;
  msg_data.grippers_angle = grippers_angle;
  msg_data.grippers_effort = grippers_effort;
  msg_data.status_code = status_code;
  msg_data.set_zero = set_zero;

  PiperMessage msg(MessageType::GRIPPER_CTRL, msg_data);
  CanFrameMsg can_frame;

  if (protocol_->encode_message(msg, can_frame)) {
    return can_interface_->send_message(can_frame);
  }

  return false;
}

bool PiperController::move_c_axis_update(uint8_t instruction_num)
{
  if (!is_connected()) {
    return false;
  }

  MsgCircularPatternCoordUpdate msg_data;
  msg_data.instruction_num = instruction_num;

  PiperMessage msg(MessageType::CIRCULAR_PATTERN_COORD_UPDATE, msg_data);
  CanFrameMsg can_frame;

  if (protocol_->encode_message(msg, can_frame)) {
    return can_interface_->send_message(can_frame);
  }

  return false;
}

bool PiperController::configure_joint(
  uint8_t joint_id, uint8_t enable_pos_lim, uint8_t enable_vel_lim, int max_joint_acc)
{
  if (!is_connected()) {
    return false;
  }

  MsgJointConfig msg_data;
  msg_data.joint_num = joint_id;
  msg_data.set_zero = enable_pos_lim;
  msg_data.acc_param_is_effective = enable_vel_lim;
  msg_data.max_joint_acc = max_joint_acc;

  PiperMessage msg(MessageType::JOINT_CONFIG, msg_data);
  CanFrameMsg can_frame;

  if (protocol_->encode_message(msg, can_frame)) {
    return can_interface_->send_message(can_frame);
  }

  return false;
}

bool PiperController::configure_crash_protection(
  uint8_t j1_level, uint8_t j2_level, uint8_t j3_level, uint8_t j4_level, uint8_t j5_level,
  uint8_t j6_level)
{
  if (!is_connected()) {
    return false;
  }

  MsgCrashProtectionConfig msg_data;
  msg_data.j1_level = j1_level;
  msg_data.j2_level = j2_level;
  msg_data.j3_level = j3_level;
  msg_data.j4_level = j4_level;
  msg_data.j5_level = j5_level;
  msg_data.j6_level = j6_level;

  PiperMessage msg(MessageType::CRASH_PROTECTION_CONFIG, msg_data);
  CanFrameMsg can_frame;

  if (protocol_->encode_message(msg, can_frame)) {
    return can_interface_->send_message(can_frame);
  }

  return false;
}

bool PiperController::set_master_slave_config(
  uint8_t master_slave_mode, uint8_t teach_mode, uint8_t user_value1, uint8_t user_value2)
{
  if (!is_connected()) {
    return false;
  }

  MsgMasterSlaveConfig msg_data;
  msg_data.master_slave_mode = master_slave_mode;
  msg_data.teach_mode = teach_mode;
  msg_data.user_value1 = user_value1;
  msg_data.user_value2 = user_value2;

  PiperMessage msg(MessageType::MASTER_SLAVE_CONFIG, msg_data);
  CanFrameMsg can_frame;

  if (protocol_->encode_message(msg, can_frame)) {
    return can_interface_->send_message(can_frame);
  }

  return false;
}

bool PiperController::motion_control_1(
  uint8_t emergency_stop, uint8_t track_ctrl, uint8_t grag_teach_ctrl, uint8_t trajectory_index,
  uint16_t name_index)
{
  if (!is_connected()) {
    return false;
  }

  MsgMotionCtrl1 msg_data;
  msg_data.emergency_stop = emergency_stop;
  msg_data.track_ctrl = track_ctrl;
  msg_data.grag_teach_ctrl = grag_teach_ctrl;
  msg_data.trajectory_index = trajectory_index;
  msg_data.name_index = name_index;

  PiperMessage msg(MessageType::MOTION_CTRL_1, msg_data);
  CanFrameMsg can_frame;

  if (protocol_->encode_message(msg, can_frame)) {
    return can_interface_->send_message(can_frame);
  }

  return false;
}

bool PiperController::motion_control_2(
  uint8_t ctrl_mode, uint8_t move_mode, uint8_t move_speed_rate, uint8_t is_mit_mode,
  uint8_t residence_time, uint8_t installation_pos)
{
  if (!is_connected()) {
    return false;
  }

  MsgMotionCtrl2 msg_data;
  msg_data.ctrl_mode = ctrl_mode;
  msg_data.move_mode = move_mode;
  msg_data.move_speed_rate = move_speed_rate;
  msg_data.is_mit_mode = is_mit_mode;
  msg_data.residence_time = residence_time;
  msg_data.installation_pos = installation_pos;

  PiperMessage msg(MessageType::MOTION_CTRL_2, msg_data);
  CanFrameMsg can_frame;

  if (protocol_->encode_message(msg, can_frame)) {
    return can_interface_->send_message(can_frame);
  }

  return false;
}

std::string PiperController::get_firmware_version()
{
  std::lock_guard<std::mutex> lock(firmware_mutex_);

  std::string version;
  for (auto byte : firmware_data_) {
    if (byte != 0) {
      version.push_back(static_cast<char>(byte));
    }
  }

  return version.empty() ? "Unknown" : version;
}

std::pair<double, double> PiperController::get_sdk_joint_limit_param(const std::string & joint_name)
{
  if (parameters_) {
    // Use the updated non-deprecated method
    const auto limits = parameters_->get_joint_limits(joint_name);
    return std::pair<double, double>(limits.min, limits.max);
  }
  return std::pair<double, double>(-M_PI, M_PI);
}

std::pair<double, double> PiperController::get_sdk_gripper_range_param()
{
  if (parameters_) {
    // Use the updated non-deprecated method
    const auto range = parameters_->get_gripper_range();
    return std::pair<double, double>(range.min, range.max);
  }
  return std::pair<double, double>(0.0, 0.08);
}

void PiperController::set_sdk_joint_limit_param(
  const std::string & joint_name, double min_val, double max_val)
{
  if (parameters_) {
    // Use the updated non-deprecated method
    parameters_->set_joint_limits(joint_name, min_val, max_val);
  }
}

void PiperController::set_sdk_gripper_range_param(double min_val, double max_val)
{
  if (parameters_) {
    // Use the updated non-deprecated method
    parameters_->set_gripper_range(min_val, max_val);
  }
}

void PiperController::parse_can_frame(CanFrameMsg & frame)
{
  PiperMessage msg;

  if (protocol_->decode_message(frame, msg)) {
    // Handle decoded message based on type
    switch (msg.get_type()) {
      case MessageType::ARM_STATUS:
        if (auto data = std::get_if<MsgArmStatusFeedback>(&msg.get_data())) {
          std::lock_guard<std::mutex> lock(arm_status_mutex_);
          arm_status_.ctrl_mode = data->ctrl_mode;
          arm_status_.arm_status = data->arm_status;
          arm_status_.mode_feed = data->mode_feed;
          arm_status_.teach_status = data->teach_status;
          arm_status_.motion_status = data->motion_status;
          arm_status_.trajectory_num = data->trajectory_num;
          arm_status_.err_code_comm = data->err_code_comm;
          arm_status_.err_code_angle = data->err_code_angle;
        }
        break;

      case MessageType::END_POSE_XY:
        if (auto data = std::get_if<MsgCartesianCtrl>(&msg.get_data())) {
          std::lock_guard<std::mutex> lock(arm_end_pose_mutex_);
          arm_end_pose_.x = data->axis1;
          arm_end_pose_.y = data->axis2;
        }
        break;

      case MessageType::END_POSE_ZRX:
        if (auto data = std::get_if<MsgCartesianCtrl>(&msg.get_data())) {
          std::lock_guard<std::mutex> lock(arm_end_pose_mutex_);
          arm_end_pose_.z = data->axis1;
          arm_end_pose_.rx = data->axis2;
        }
        break;

      case MessageType::END_POSE_RYRZ:
        if (auto data = std::get_if<MsgCartesianCtrl>(&msg.get_data())) {
          std::lock_guard<std::mutex> lock(arm_end_pose_mutex_);
          arm_end_pose_.ry = data->axis1;
          arm_end_pose_.rz = data->axis2;
        }
        break;

      case MessageType::JOINT_12:
        if (auto data = std::get_if<MsgJointCtrl12>(&msg.get_data())) {
          std::lock_guard<std::mutex> lock(arm_joint_mutex_);
          arm_joint_.j1 = data->joint_1;
          arm_joint_.j2 = data->joint_2;
        }
        break;

      case MessageType::JOINT_34:
        if (auto data = std::get_if<MsgJointCtrl34>(&msg.get_data())) {
          std::lock_guard<std::mutex> lock(arm_joint_mutex_);
          arm_joint_.j3 = data->joint_3;
          arm_joint_.j4 = data->joint_4;
        }
        break;

      case MessageType::JOINT_56:
        if (auto data = std::get_if<MsgJointCtrl56>(&msg.get_data())) {
          std::lock_guard<std::mutex> lock(arm_joint_mutex_);
          arm_joint_.j5 = data->joint_5;
          arm_joint_.j6 = data->joint_6;
        }
        break;

      case MessageType::GRIPPER:
        if (auto data = std::get_if<MsgGripperCtrl>(&msg.get_data())) {
          std::lock_guard<std::mutex> lock(arm_gripper_mutex_);
          arm_gripper_.grippers_angle = data->grippers_angle;
          arm_gripper_.grippers_effort = data->grippers_effort;
          arm_gripper_.status_code = data->status_code;
          arm_gripper_.set_zero = data->set_zero;
        }
        break;

      case MessageType::FIRMWARE_VERSION:
        // Store firmware version bytes properly
        {
          std::lock_guard<std::mutex> lock(firmware_mutex_);
          if (auto data = std::get_if<MsgFirmwareVersion>(&msg.get_data())) {
            firmware_data_.assign(data->version_data, data->version_data + 8);
          }
        }
        break;

      case MessageType::MOTOR_INFO_HIGH_SPD:
        // Figure out which joint this high speed info belongs to based on CAN ID
        {
          uint8_t joint_num = 0;

          if (
            (frame.arbitration_id >=
             static_cast<uint32_t>(CanIdPiper::ARM_INFO_HIGH_SPD_FEEDBACK_1)) &&
            (frame.arbitration_id <=
             static_cast<uint32_t>(CanIdPiper::ARM_INFO_HIGH_SPD_FEEDBACK_6))) {
            joint_num = frame.arbitration_id -
                        static_cast<uint32_t>(CanIdPiper::ARM_INFO_HIGH_SPD_FEEDBACK_1) + 1;

            if (joint_num >= 1 && joint_num <= 6) {
              if (auto data = std::get_if<MsgArmHighSpeedFeedback>(&msg.get_data())) {
                std::lock_guard<std::mutex> lock(motor_info_mutex_);
                motor_info_[joint_num - 1].position = data->position;
                motor_info_[joint_num - 1].motor_speed = data->motor_speed;
                motor_info_[joint_num - 1].current = data->current;
              }
            }
          }
        }
        break;

      case MessageType::MOTOR_INFO_LOW_SPD:
        // Figure out which joint this low speed info belongs to based on CAN ID
        {
          uint8_t joint_num = 0;

          if (
            (frame.arbitration_id >=
             static_cast<uint32_t>(CanIdPiper::ARM_INFO_LOW_SPD_FEEDBACK_1)) &&
            (frame.arbitration_id <=
             static_cast<uint32_t>(CanIdPiper::ARM_INFO_LOW_SPD_FEEDBACK_6))) {
            joint_num = frame.arbitration_id -
                        static_cast<uint32_t>(CanIdPiper::ARM_INFO_LOW_SPD_FEEDBACK_1) + 1;

            if (joint_num >= 1 && joint_num <= 6) {
              if (auto data = std::get_if<MsgArmLowSpeedFeedback>(&msg.get_data())) {
                std::lock_guard<std::mutex> lock(motor_info_mutex_);
                motor_info_[joint_num - 1].voltage = data->voltage;
                motor_info_[joint_num - 1].drive_temperature = data->drive_temperature;
                motor_info_[joint_num - 1].motor_temperature = data->motor_temperature;
                motor_info_[joint_num - 1].drive_status = data->drive_status;
                motor_info_[joint_num - 1].bus_current = data->bus_current;
              }
            }
          }
        }
        break;

      default:
        // Ignore other message types
        break;
    }
  }
}

void PiperController::read_can_loop()
{
  while (running_) {
    // This loop is empty because we use the callback mechanism for reading CAN frames
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}

void PiperController::can_monitor_loop()
{
  while (running_) {
    // Periodically check CAN connection state
    if (!can_interface_ || !can_interface_->is_running()) {
      // Try to reconnect
      if (can_interface_) {
        can_interface_->stop();
      }
      if (auto_init_) {
        connect_port(true);
      }
    }

    // Sleep to prevent high CPU usage
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
}

bool PiperController::cartesian_ctrl_xy(int x, int y)
{
  MsgCartesianCtrl msg_data;
  msg_data.axis1 = x;
  msg_data.axis2 = y;

  PiperMessage msg(MessageType::CARTESIAN_CTRL_1, msg_data);
  CanFrameMsg can_frame;

  if (!protocol_->encode_message(msg, can_frame)) {
    return false;
  }

  return can_interface_->send_message(can_frame);
}

bool PiperController::cartesian_ctrl_zrx(int z, int rx)
{
  MsgCartesianCtrl msg_data;
  msg_data.axis1 = z;
  msg_data.axis2 = rx;

  PiperMessage msg(MessageType::CARTESIAN_CTRL_2, msg_data);
  CanFrameMsg can_frame;

  if (!protocol_->encode_message(msg, can_frame)) {
    return false;
  }

  return can_interface_->send_message(can_frame);
}

bool PiperController::cartesian_ctrl_ryrz(int ry, int rz)
{
  MsgCartesianCtrl msg_data;
  msg_data.axis1 = ry;
  msg_data.axis2 = rz;

  PiperMessage msg(MessageType::CARTESIAN_CTRL_3, msg_data);
  CanFrameMsg can_frame;

  if (!protocol_->encode_message(msg, can_frame)) {
    return false;
  }

  return can_interface_->send_message(can_frame);
}

bool PiperController::joint_ctrl_12(int j1, int j2)
{
  MsgJointCtrl12 msg_data;
  msg_data.joint_1 = j1;
  msg_data.joint_2 = j2;

  PiperMessage msg(MessageType::JOINT_CTRL_12, msg_data);
  CanFrameMsg can_frame;

  if (!protocol_->encode_message(msg, can_frame)) {
    return false;
  }

  return can_interface_->send_message(can_frame);
}

bool PiperController::joint_ctrl_34(int j3, int j4)
{
  MsgJointCtrl34 msg_data;
  msg_data.joint_3 = j3;
  msg_data.joint_4 = j4;

  PiperMessage msg(MessageType::JOINT_CTRL_34, msg_data);
  CanFrameMsg can_frame;

  if (!protocol_->encode_message(msg, can_frame)) {
    return false;
  }

  return can_interface_->send_message(can_frame);
}

bool PiperController::joint_ctrl_56(int j5, int j6)
{
  MsgJointCtrl56 msg_data;
  msg_data.joint_5 = j5;
  msg_data.joint_6 = j6;

  PiperMessage msg(MessageType::JOINT_CTRL_56, msg_data);
  CanFrameMsg can_frame;

  if (!protocol_->encode_message(msg, can_frame)) {
    return false;
  }

  return can_interface_->send_message(can_frame);
}

int PiperController::check_joint_sdk_limit(int joint_value, const std::string & joint_name)
{
  if (!parameters_) {
    return joint_value;
  }

  // Get joint limits in radians using the non-deprecated method
  auto limits = parameters_->get_joint_limits(joint_name);

  // Convert the joint value from protocol units (0.001 degrees) to radians
  double joint_rad = joint_value * 0.001 * M_PI / 180.0;

  // Check limits
  if (joint_rad < limits.min) {
    joint_rad = limits.min;
  } else if (joint_rad > limits.max) {
    joint_rad = limits.max;
  }

  // Convert back to protocol units (0.001 degrees)
  return static_cast<int>(joint_rad * 180.0 / M_PI * 1000.0);
}

MotorInfo PiperController::get_motor_info(uint8_t joint_num) const
{
  if (joint_num < 1 || joint_num > 6) {
    // Invalid joint number, return empty info
    return MotorInfo{};
  }

  std::lock_guard<std::mutex> lock(motor_info_mutex_);
  return motor_info_[joint_num - 1];  // Array is 0-indexed, but joints are 1-indexed
}

}  // namespace piper
}  // namespace agilex