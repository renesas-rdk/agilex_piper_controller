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
#include "agilex_piper_controller/piper_protocol.hpp"

#include <cstring>
#include <variant>

namespace agilex
{
namespace piper
{

// Static methods for byte conversion in PiperProtocolBase
int32_t PiperProtocolBase::bytes_to_int32(const uint8_t * data)
{
  int32_t value = 0;
  value |= static_cast<int32_t>(data[0]) << 24;
  value |= static_cast<int32_t>(data[1]) << 16;
  value |= static_cast<int32_t>(data[2]) << 8;
  value |= static_cast<int32_t>(data[3]);
  return value;
}

int16_t PiperProtocolBase::bytes_to_int16(const uint8_t * data)
{
  int16_t value = 0;
  value |= static_cast<int16_t>(data[0]) << 8;
  value |= static_cast<int16_t>(data[1]);
  return value;
}

uint32_t PiperProtocolBase::bytes_to_uint32(const uint8_t * data)
{
  uint32_t value = 0;
  value |= static_cast<uint32_t>(data[0]) << 24;
  value |= static_cast<uint32_t>(data[1]) << 16;
  value |= static_cast<uint32_t>(data[2]) << 8;
  value |= static_cast<uint32_t>(data[3]);
  return value;
}

uint16_t PiperProtocolBase::bytes_to_uint16(const uint8_t * data)
{
  uint16_t value = 0;
  value |= static_cast<uint16_t>(data[0]) << 8;
  value |= static_cast<uint16_t>(data[1]);
  return value;
}

void PiperProtocolBase::int32_to_bytes(int32_t value, uint8_t * data)
{
  data[0] = static_cast<uint8_t>((value >> 24) & 0xFF);
  data[1] = static_cast<uint8_t>((value >> 16) & 0xFF);
  data[2] = static_cast<uint8_t>((value >> 8) & 0xFF);
  data[3] = static_cast<uint8_t>(value & 0xFF);
}

void PiperProtocolBase::int16_to_bytes(int16_t value, uint8_t * data)
{
  data[0] = static_cast<uint8_t>((value >> 8) & 0xFF);
  data[1] = static_cast<uint8_t>(value & 0xFF);
}

void PiperProtocolBase::uint32_to_bytes(uint32_t value, uint8_t * data)
{
  data[0] = static_cast<uint8_t>((value >> 24) & 0xFF);
  data[1] = static_cast<uint8_t>((value >> 16) & 0xFF);
  data[2] = static_cast<uint8_t>((value >> 8) & 0xFF);
  data[3] = static_cast<uint8_t>(value & 0xFF);
}

void PiperProtocolBase::uint16_to_bytes(uint16_t value, uint8_t * data)
{
  data[0] = static_cast<uint8_t>((value >> 8) & 0xFF);
  data[1] = static_cast<uint8_t>(value & 0xFF);
}

// PiperProtocolV2 implementation
PiperProtocolV2::PiperProtocolV2() {}

ProtocolVersion PiperProtocolV2::get_protocol_version() const { return ProtocolVersion::V2; }

bool PiperProtocolV2::encode_message(const PiperMessage & msg, CanFrameMsg & frame)
{
  CanIdPiper can_id = PiperMessage::message_type_to_can_id(msg.get_type());
  frame.arbitration_id = static_cast<uint32_t>(can_id);
  frame.is_extended_id = false;
  frame.dlc = 8;

  // Initialize data to zeros
  memset(frame.data, 0, 8);

  // Encode based on message type
  switch (msg.get_type()) {
    case MessageType::ENABLE_DISABLE_ARM:
      return encode_enable_disable_arm(msg, frame);
    case MessageType::MOTION_CTRL_1:
      return encode_motion_ctrl_1(msg, frame);
    case MessageType::MOTION_CTRL_2:
      return encode_motion_ctrl_2(msg, frame);
    case MessageType::CARTESIAN_CTRL_1:
      return encode_cartesian_ctrl_1(msg, frame);
    case MessageType::CARTESIAN_CTRL_2:
      return encode_cartesian_ctrl_2(msg, frame);
    case MessageType::CARTESIAN_CTRL_3:
      return encode_cartesian_ctrl_3(msg, frame);
    case MessageType::JOINT_CTRL_12:
      return encode_joint_ctrl_12(msg, frame);
    case MessageType::JOINT_CTRL_34:
      return encode_joint_ctrl_34(msg, frame);
    case MessageType::JOINT_CTRL_56:
      return encode_joint_ctrl_56(msg, frame);
    case MessageType::GRIPPER_CTRL:
      return encode_gripper_ctrl(msg, frame);
    case MessageType::JOINT_CONFIG:
      return encode_joint_config(msg, frame);
    case MessageType::CRASH_PROTECTION_CONFIG:
      return encode_crash_protection_config(msg, frame);
    case MessageType::MASTER_SLAVE_CONFIG:
      return encode_master_slave_config(msg, frame);
    case MessageType::CIRCULAR_PATTERN_COORD_UPDATE:
      return encode_circular_pattern_coord_update(msg, frame);
    default:
      return false;
  }
}

bool PiperProtocolV2::decode_message(const CanFrameMsg & frame, PiperMessage & msg)
{
  MessageType type = PiperMessage::can_id_to_message_type(frame.arbitration_id);
  msg.set_type(type);

  switch (type) {
    case MessageType::ARM_STATUS:
      return decode_arm_status(frame, msg);
    case MessageType::END_POSE_XY:
      return decode_end_pose_xy(frame, msg);
    case MessageType::END_POSE_ZRX:
      return decode_end_pose_zrx(frame, msg);
    case MessageType::END_POSE_RYRZ:
      return decode_end_pose_ryrz(frame, msg);
    case MessageType::JOINT_12:
      return decode_joint_12(frame, msg);
    case MessageType::JOINT_34:
      return decode_joint_34(frame, msg);
    case MessageType::JOINT_56:
      return decode_joint_56(frame, msg);
    case MessageType::GRIPPER:
      return decode_gripper(frame, msg);
    case MessageType::MOTOR_INFO_HIGH_SPD:
      return decode_motor_info_high_spd(frame, msg);
    case MessageType::MOTOR_INFO_LOW_SPD:
      return decode_motor_info_low_spd(frame, msg);
    case MessageType::FIRMWARE_VERSION:
      return decode_firmware_version(frame, msg);
    default:
      return false;
  }
}

// Message encoding implementations

bool PiperProtocolV2::encode_enable_disable_arm(const PiperMessage & msg, CanFrameMsg & frame)
{
  if (auto data = std::get_if<MsgEnableDisableArm>(&msg.get_data())) {
    frame.data[0] = data->motor_num;
    frame.data[1] = data->enable_flag;
    return true;
  }
  return false;
}

bool PiperProtocolV2::encode_motion_ctrl_1(const PiperMessage & msg, CanFrameMsg & frame)
{
  if (auto data = std::get_if<MsgMotionCtrl1>(&msg.get_data())) {
    frame.data[0] = data->emergency_stop;
    frame.data[1] = data->track_ctrl;
    frame.data[2] = data->grag_teach_ctrl;
    frame.data[3] = data->trajectory_index;
    PiperProtocolBase::uint16_to_bytes(data->name_index, &frame.data[4]);

    // Fill the remaining bytes with zeros (FIXME: crc checksue)
    frame.data[6] = 0x00;
    frame.data[7] = 0x00;
    return true;
  }
  return false;
}

bool PiperProtocolV2::encode_motion_ctrl_2(const PiperMessage & msg, CanFrameMsg & frame)
{
  if (auto data = std::get_if<MsgMotionCtrl2>(&msg.get_data())) {
    frame.data[0] = data->ctrl_mode;
    frame.data[1] = data->move_mode;
    frame.data[2] = data->move_speed_rate;
    frame.data[3] = data->is_mit_mode;
    frame.data[4] = data->residence_time;
    frame.data[5] = data->installation_pos;
    // Fill the remaining bytes with zeros
    frame.data[6] = 0x00;
    frame.data[7] = 0x00;
    return true;
  }
  return false;
}

bool PiperProtocolV2::encode_cartesian_ctrl_1(const PiperMessage & msg, CanFrameMsg & frame)
{
  if (auto data = std::get_if<MsgCartesianCtrl>(&msg.get_data())) {
    PiperProtocolBase::int32_to_bytes(data->axis1, &frame.data[0]);  // X axis
    PiperProtocolBase::int32_to_bytes(data->axis2, &frame.data[4]);  // Y axis
    return true;
  }
  return false;
}

bool PiperProtocolV2::encode_cartesian_ctrl_2(const PiperMessage & msg, CanFrameMsg & frame)
{
  if (auto data = std::get_if<MsgCartesianCtrl>(&msg.get_data())) {
    PiperProtocolBase::int32_to_bytes(data->axis1, &frame.data[0]);  // Z axis
    PiperProtocolBase::int32_to_bytes(data->axis2, &frame.data[4]);  // RX axis
    return true;
  }
  return false;
}

bool PiperProtocolV2::encode_cartesian_ctrl_3(const PiperMessage & msg, CanFrameMsg & frame)
{
  if (auto data = std::get_if<MsgCartesianCtrl>(&msg.get_data())) {
    PiperProtocolBase::int32_to_bytes(data->axis1, &frame.data[0]);  // RY axis
    PiperProtocolBase::int32_to_bytes(data->axis2, &frame.data[4]);  // RZ axis
    return true;
  }
  return false;
}

bool PiperProtocolV2::encode_joint_ctrl_12(const PiperMessage & msg, CanFrameMsg & frame)
{
  if (auto data = std::get_if<MsgJointCtrl12>(&msg.get_data())) {
    PiperProtocolBase::int32_to_bytes(data->joint_1, &frame.data[0]);
    PiperProtocolBase::int32_to_bytes(data->joint_2, &frame.data[4]);
    return true;
  }
  return false;
}

bool PiperProtocolV2::encode_joint_ctrl_34(const PiperMessage & msg, CanFrameMsg & frame)
{
  if (auto data = std::get_if<MsgJointCtrl34>(&msg.get_data())) {
    PiperProtocolBase::int32_to_bytes(data->joint_3, &frame.data[0]);
    PiperProtocolBase::int32_to_bytes(data->joint_4, &frame.data[4]);
    return true;
  }
  return false;
}

bool PiperProtocolV2::encode_joint_ctrl_56(const PiperMessage & msg, CanFrameMsg & frame)
{
  if (auto data = std::get_if<MsgJointCtrl56>(&msg.get_data())) {
    PiperProtocolBase::int32_to_bytes(data->joint_5, &frame.data[0]);
    PiperProtocolBase::int32_to_bytes(data->joint_6, &frame.data[4]);
    return true;
  }
  return false;
}

bool PiperProtocolV2::encode_gripper_ctrl(const PiperMessage & msg, CanFrameMsg & frame)
{
  if (auto data = std::get_if<MsgGripperCtrl>(&msg.get_data())) {
    PiperProtocolBase::int32_to_bytes(data->grippers_angle, &frame.data[0]);
    PiperProtocolBase::uint16_to_bytes(data->grippers_effort, &frame.data[4]);
    frame.data[6] = data->status_code;
    frame.data[7] = data->set_zero;
    return true;
  }
  return false;
}

bool PiperProtocolV2::encode_joint_config(const PiperMessage & msg, CanFrameMsg & frame)
{
  if (auto data = std::get_if<MsgJointConfig>(&msg.get_data())) {
    frame.data[0] = data->joint_num;
    frame.data[1] = data->set_zero;
    frame.data[2] = data->acc_param_is_effective;
    PiperProtocolBase::uint16_to_bytes(
      data->max_joint_acc, &frame.data[3]);  // Changed from position 4 to position 3
    frame.data[5] = data->clear_joint_err;   // Added field
    frame.data[6] = 0x00;                    // Added zero padding
    frame.data[7] = 0x00;                    // Added zero padding
    return true;
  }
  return false;
}

bool PiperProtocolV2::encode_crash_protection_config(const PiperMessage & msg, CanFrameMsg & frame)
{
  if (auto data = std::get_if<MsgCrashProtectionConfig>(&msg.get_data())) {
    frame.data[0] = data->j1_level;
    frame.data[1] = data->j2_level;
    frame.data[2] = data->j3_level;
    frame.data[3] = data->j4_level;
    frame.data[4] = data->j5_level;
    frame.data[5] = data->j6_level;
    frame.data[6] = 0x00;
    frame.data[7] = 0x00;
    return true;
  }
  return false;
}

bool PiperProtocolV2::encode_master_slave_config(const PiperMessage & msg, CanFrameMsg & frame)
{
  if (auto data = std::get_if<MsgMasterSlaveConfig>(&msg.get_data())) {
    frame.data[0] = data->master_slave_mode;
    frame.data[1] = data->teach_mode;
    frame.data[2] = data->user_value1;
    frame.data[3] = data->user_value2;
    memset(&frame.data[4], 0, 4);
    return true;
  }
  return false;
}

bool PiperProtocolV2::encode_circular_pattern_coord_update(
  const PiperMessage & msg, CanFrameMsg & frame)
{
  if (auto data = std::get_if<MsgCircularPatternCoordUpdate>(&msg.get_data())) {
    frame.data[0] = data->instruction_num;
    return true;
  }
  return false;
}

// Message decoding implementations

bool PiperProtocolV2::decode_arm_status(const CanFrameMsg & frame, PiperMessage & msg)
{
  MsgArmStatusFeedback arm_status;
  arm_status.ctrl_mode = frame.data[0];
  arm_status.arm_status = frame.data[1];
  arm_status.mode_feed = frame.data[2];
  arm_status.teach_status = frame.data[3];
  arm_status.motion_status = frame.data[4];
  arm_status.trajectory_num = frame.data[5];
  arm_status.err_code_comm = frame.data[6];
  arm_status.err_code_angle = frame.data[7];
  msg.set_data(arm_status);
  return true;
}

bool PiperProtocolV2::decode_end_pose_xy(const CanFrameMsg & frame, PiperMessage & msg)
{
  int32_t x = PiperProtocolBase::bytes_to_int32(&frame.data[0]);
  int32_t y = PiperProtocolBase::bytes_to_int32(&frame.data[4]);

  MsgCartesianCtrl end_pose_xy;
  end_pose_xy.axis1 = x;
  end_pose_xy.axis2 = y;
  msg.set_data(end_pose_xy);
  return true;
}

bool PiperProtocolV2::decode_end_pose_zrx(const CanFrameMsg & frame, PiperMessage & msg)
{
  int32_t z = PiperProtocolBase::bytes_to_int32(&frame.data[0]);
  int32_t rx = PiperProtocolBase::bytes_to_int32(&frame.data[4]);

  MsgCartesianCtrl end_pose_zrx;
  end_pose_zrx.axis1 = z;
  end_pose_zrx.axis2 = rx;
  msg.set_data(end_pose_zrx);
  return true;
}

bool PiperProtocolV2::decode_end_pose_ryrz(const CanFrameMsg & frame, PiperMessage & msg)
{
  int32_t ry = PiperProtocolBase::bytes_to_int32(&frame.data[0]);
  int32_t rz = PiperProtocolBase::bytes_to_int32(&frame.data[4]);

  MsgCartesianCtrl end_pose_ryrz;
  end_pose_ryrz.axis1 = ry;
  end_pose_ryrz.axis2 = rz;
  msg.set_data(end_pose_ryrz);
  return true;
}

bool PiperProtocolV2::decode_joint_12(const CanFrameMsg & frame, PiperMessage & msg)
{
  MsgJointCtrl12 joints;
  joints.joint_1 = PiperProtocolBase::bytes_to_int32(&frame.data[0]);
  joints.joint_2 = PiperProtocolBase::bytes_to_int32(&frame.data[4]);
  msg.set_data(joints);
  return true;
}

bool PiperProtocolV2::decode_joint_34(const CanFrameMsg & frame, PiperMessage & msg)
{
  MsgJointCtrl34 joints;
  joints.joint_3 = PiperProtocolBase::bytes_to_int32(&frame.data[0]);
  joints.joint_4 = PiperProtocolBase::bytes_to_int32(&frame.data[4]);
  msg.set_data(joints);
  return true;
}

bool PiperProtocolV2::decode_joint_56(const CanFrameMsg & frame, PiperMessage & msg)
{
  MsgJointCtrl56 joints;
  joints.joint_5 = PiperProtocolBase::bytes_to_int32(&frame.data[0]);
  joints.joint_6 = PiperProtocolBase::bytes_to_int32(&frame.data[4]);
  msg.set_data(joints);
  return true;
}

bool PiperProtocolV2::decode_gripper(const CanFrameMsg & frame, PiperMessage & msg)
{
  MsgGripperCtrl gripper;
  gripper.grippers_angle = PiperProtocolBase::bytes_to_int32(&frame.data[0]);
  gripper.grippers_effort = PiperProtocolBase::bytes_to_uint16(&frame.data[4]);
  gripper.status_code = frame.data[6];
  gripper.set_zero = frame.data[7];
  msg.set_data(gripper);
  return true;
}

bool PiperProtocolV2::decode_motor_info_high_spd(const CanFrameMsg & frame, PiperMessage & msg)
{
  MsgArmHighSpeedFeedback motor_info;
  motor_info.motor_speed = PiperProtocolBase::bytes_to_int16(&frame.data[0]);
  motor_info.current = PiperProtocolBase::bytes_to_uint16(&frame.data[2]);
  motor_info.position = PiperProtocolBase::bytes_to_int32(&frame.data[4]);
  msg.set_data(motor_info);
  return true;
}

bool PiperProtocolV2::decode_motor_info_low_spd(const CanFrameMsg & frame, PiperMessage & msg)
{
  MsgArmLowSpeedFeedback motor_info;
  motor_info.voltage = PiperProtocolBase::bytes_to_uint16(&frame.data[0]);
  motor_info.drive_temperature = PiperProtocolBase::bytes_to_int16(&frame.data[2]);
  motor_info.motor_temperature = frame.data[4];
  motor_info.drive_status = frame.data[5];
  motor_info.bus_current = PiperProtocolBase::bytes_to_uint16(&frame.data[6]);
  msg.set_data(motor_info);
  return true;
}

bool PiperProtocolV2::decode_firmware_version(const CanFrameMsg & frame, PiperMessage & msg)
{
  MsgFirmwareVersion firmware_version;
  memcpy(firmware_version.version_data, frame.data, 8);
  msg.set_data(firmware_version);
  return true;
}

}  // namespace piper
}  // namespace agilex