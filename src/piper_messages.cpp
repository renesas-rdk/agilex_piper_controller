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
#include "agilex_piper_controller/piper_messages.hpp"

namespace agilex
{
namespace piper
{

PiperMessage::PiperMessage(MessageType type) : type_(type), data_(std::monostate{}) {}

PiperMessage::PiperMessage(MessageType type, const PiperMessageData & data)
: type_(type), data_(data)
{
}

CanIdPiper PiperMessage::message_type_to_can_id(MessageType type)
{
  switch (type) {
    case MessageType::ENABLE_DISABLE_ARM:
      return CanIdPiper::ARM_MOTOR_ENABLE_DISABLE_CONFIG;
    case MessageType::MOTION_CTRL_1:
      return CanIdPiper::ARM_MOTION_CTRL_1;
    case MessageType::MOTION_CTRL_2:
      return CanIdPiper::ARM_MOTION_CTRL_2;
    case MessageType::CARTESIAN_CTRL_1:
      return CanIdPiper::ARM_MOTION_CTRL_CARTESIAN_1;
    case MessageType::CARTESIAN_CTRL_2:
      return CanIdPiper::ARM_MOTION_CTRL_CARTESIAN_2;
    case MessageType::CARTESIAN_CTRL_3:
      return CanIdPiper::ARM_MOTION_CTRL_CARTESIAN_3;
    case MessageType::JOINT_CTRL_12:
      return CanIdPiper::ARM_JOINT_CTRL_12;
    case MessageType::JOINT_CTRL_34:
      return CanIdPiper::ARM_JOINT_CTRL_34;
    case MessageType::JOINT_CTRL_56:
      return CanIdPiper::ARM_JOINT_CTRL_56;
    case MessageType::GRIPPER_CTRL:
      return CanIdPiper::ARM_GRIPPER_CTRL;
    case MessageType::JOINT_CONFIG:
      return CanIdPiper::ARM_JOINT_CONFIG;
    case MessageType::CRASH_PROTECTION_CONFIG:
      return CanIdPiper::ARM_CRASH_PROTECTION_RATING_CONFIG;
    case MessageType::MASTER_SLAVE_CONFIG:
      return CanIdPiper::ARM_MASTER_SLAVE_MODE_CONFIG;
    case MessageType::CIRCULAR_PATTERN_COORD_UPDATE:
      return CanIdPiper::ARM_CIRCULAR_PATTERN_COORD_NUM_UPDATE_CTRL;
    case MessageType::PARAM_ENQUIRY_CONFIG:
      return CanIdPiper::ARM_PARAM_ENQUIRY_AND_CONFIG;
    case MessageType::LIGHT_CTRL:
      return CanIdPiper::ARM_LIGHT_CTRL;
    case MessageType::ARM_STATUS:
      return CanIdPiper::ARM_STATUS_FEEDBACK;
    case MessageType::END_POSE_XY:
      return CanIdPiper::ARM_END_POSE_XY_FEEDBACK;
    case MessageType::END_POSE_ZRX:
      return CanIdPiper::ARM_END_POSE_ZRX_FEEDBACK;
    case MessageType::END_POSE_RYRZ:
      return CanIdPiper::ARM_END_POSE_RYRZ_FEEDBACK;
    case MessageType::JOINT_12:
      return CanIdPiper::ARM_JOINT_FEEDBACK_12;
    case MessageType::JOINT_34:
      return CanIdPiper::ARM_JOINT_FEEDBACK_34;
    case MessageType::JOINT_56:
      return CanIdPiper::ARM_JOINT_FEEDBACK_56;
    case MessageType::GRIPPER:
      return CanIdPiper::ARM_GRIPPER_FEEDBACK;
    case MessageType::FIRMWARE_VERSION:
      return CanIdPiper::ARM_FIRMWARE_READ;
    case MessageType::JOINT_MIT_CTRL_1:
      return CanIdPiper::ARM_JOINT_MIT_CTRL_1;
    case MessageType::JOINT_MIT_CTRL_2:
      return CanIdPiper::ARM_JOINT_MIT_CTRL_2;
    case MessageType::JOINT_MIT_CTRL_3:
      return CanIdPiper::ARM_JOINT_MIT_CTRL_3;
    case MessageType::JOINT_MIT_CTRL_4:
      return CanIdPiper::ARM_JOINT_MIT_CTRL_4;
    case MessageType::JOINT_MIT_CTRL_5:
      return CanIdPiper::ARM_JOINT_MIT_CTRL_5;
    case MessageType::JOINT_MIT_CTRL_6:
      return CanIdPiper::ARM_JOINT_MIT_CTRL_6;
    case MessageType::JOINT_VEL_ACC_1:
      return CanIdPiper::ARM_FEEDBACK_JOINT_VEL_ACC_1;
    case MessageType::JOINT_VEL_ACC_2:
      return CanIdPiper::ARM_FEEDBACK_JOINT_VEL_ACC_2;
    case MessageType::JOINT_VEL_ACC_3:
      return CanIdPiper::ARM_FEEDBACK_JOINT_VEL_ACC_3;
    case MessageType::JOINT_VEL_ACC_4:
      return CanIdPiper::ARM_FEEDBACK_JOINT_VEL_ACC_4;
    case MessageType::JOINT_VEL_ACC_5:
      return CanIdPiper::ARM_FEEDBACK_JOINT_VEL_ACC_5;
    case MessageType::JOINT_VEL_ACC_6:
      return CanIdPiper::ARM_FEEDBACK_JOINT_VEL_ACC_6;
    default:
      return CanIdPiper::ARM_STATUS_FEEDBACK;
  }
}

MessageType PiperMessage::can_id_to_message_type(uint32_t can_id)
{
  switch (can_id) {
    case static_cast<uint32_t>(CanIdPiper::ARM_MOTOR_ENABLE_DISABLE_CONFIG):
      return MessageType::ENABLE_DISABLE_ARM;
    case static_cast<uint32_t>(CanIdPiper::ARM_MOTION_CTRL_1):
      return MessageType::MOTION_CTRL_1;
    case static_cast<uint32_t>(CanIdPiper::ARM_MOTION_CTRL_2):
      return MessageType::MOTION_CTRL_2;
    case static_cast<uint32_t>(CanIdPiper::ARM_MOTION_CTRL_CARTESIAN_1):
      return MessageType::CARTESIAN_CTRL_1;
    case static_cast<uint32_t>(CanIdPiper::ARM_MOTION_CTRL_CARTESIAN_2):
      return MessageType::CARTESIAN_CTRL_2;
    case static_cast<uint32_t>(CanIdPiper::ARM_MOTION_CTRL_CARTESIAN_3):
      return MessageType::CARTESIAN_CTRL_3;
    case static_cast<uint32_t>(CanIdPiper::ARM_JOINT_CTRL_12):
      return MessageType::JOINT_CTRL_12;
    case static_cast<uint32_t>(CanIdPiper::ARM_JOINT_CTRL_34):
      return MessageType::JOINT_CTRL_34;
    case static_cast<uint32_t>(CanIdPiper::ARM_JOINT_CTRL_56):
      return MessageType::JOINT_CTRL_56;
    case static_cast<uint32_t>(CanIdPiper::ARM_GRIPPER_CTRL):
      return MessageType::GRIPPER_CTRL;
    case static_cast<uint32_t>(CanIdPiper::ARM_JOINT_CONFIG):
      return MessageType::JOINT_CONFIG;
    case static_cast<uint32_t>(CanIdPiper::ARM_CRASH_PROTECTION_RATING_CONFIG):
      return MessageType::CRASH_PROTECTION_CONFIG;
    case static_cast<uint32_t>(CanIdPiper::ARM_MASTER_SLAVE_MODE_CONFIG):
      return MessageType::MASTER_SLAVE_CONFIG;
    case static_cast<uint32_t>(CanIdPiper::ARM_LIGHT_CTRL):
      return MessageType::LIGHT_CTRL;
    case static_cast<uint32_t>(CanIdPiper::ARM_CIRCULAR_PATTERN_COORD_NUM_UPDATE_CTRL):
      return MessageType::CIRCULAR_PATTERN_COORD_UPDATE;
    case static_cast<uint32_t>(CanIdPiper::ARM_PARAM_ENQUIRY_AND_CONFIG):
      return MessageType::PARAM_ENQUIRY_CONFIG;
    case static_cast<uint32_t>(CanIdPiper::ARM_STATUS_FEEDBACK):
      return MessageType::ARM_STATUS;
    case static_cast<uint32_t>(CanIdPiper::ARM_END_POSE_XY_FEEDBACK):
      return MessageType::END_POSE_XY;
    case static_cast<uint32_t>(CanIdPiper::ARM_END_POSE_ZRX_FEEDBACK):
      return MessageType::END_POSE_ZRX;
    case static_cast<uint32_t>(CanIdPiper::ARM_END_POSE_RYRZ_FEEDBACK):
      return MessageType::END_POSE_RYRZ;
    case static_cast<uint32_t>(CanIdPiper::ARM_JOINT_FEEDBACK_12):
      return MessageType::JOINT_12;
    case static_cast<uint32_t>(CanIdPiper::ARM_JOINT_FEEDBACK_34):
      return MessageType::JOINT_34;
    case static_cast<uint32_t>(CanIdPiper::ARM_JOINT_FEEDBACK_56):
      return MessageType::JOINT_56;
    case static_cast<uint32_t>(CanIdPiper::ARM_GRIPPER_FEEDBACK):
      return MessageType::GRIPPER;
    // Motor info high speed
    case static_cast<uint32_t>(CanIdPiper::ARM_INFO_HIGH_SPD_FEEDBACK_1):
    case static_cast<uint32_t>(CanIdPiper::ARM_INFO_HIGH_SPD_FEEDBACK_2):
    case static_cast<uint32_t>(CanIdPiper::ARM_INFO_HIGH_SPD_FEEDBACK_3):
    case static_cast<uint32_t>(CanIdPiper::ARM_INFO_HIGH_SPD_FEEDBACK_4):
    case static_cast<uint32_t>(CanIdPiper::ARM_INFO_HIGH_SPD_FEEDBACK_5):
    case static_cast<uint32_t>(CanIdPiper::ARM_INFO_HIGH_SPD_FEEDBACK_6):
      return MessageType::MOTOR_INFO_HIGH_SPD;
    // Motor info low speed
    case static_cast<uint32_t>(CanIdPiper::ARM_INFO_LOW_SPD_FEEDBACK_1):
    case static_cast<uint32_t>(CanIdPiper::ARM_INFO_LOW_SPD_FEEDBACK_2):
    case static_cast<uint32_t>(CanIdPiper::ARM_INFO_LOW_SPD_FEEDBACK_3):
    case static_cast<uint32_t>(CanIdPiper::ARM_INFO_LOW_SPD_FEEDBACK_4):
    case static_cast<uint32_t>(CanIdPiper::ARM_INFO_LOW_SPD_FEEDBACK_5):
    case static_cast<uint32_t>(CanIdPiper::ARM_INFO_LOW_SPD_FEEDBACK_6):
      return MessageType::MOTOR_INFO_LOW_SPD;
    case static_cast<uint32_t>(CanIdPiper::ARM_FIRMWARE_READ):
      return MessageType::FIRMWARE_VERSION;
    // MIT control commands
    case static_cast<uint32_t>(CanIdPiper::ARM_JOINT_MIT_CTRL_1):
      return MessageType::JOINT_MIT_CTRL_1;
    case static_cast<uint32_t>(CanIdPiper::ARM_JOINT_MIT_CTRL_2):
      return MessageType::JOINT_MIT_CTRL_2;
    case static_cast<uint32_t>(CanIdPiper::ARM_JOINT_MIT_CTRL_3):
      return MessageType::JOINT_MIT_CTRL_3;
    case static_cast<uint32_t>(CanIdPiper::ARM_JOINT_MIT_CTRL_4):
      return MessageType::JOINT_MIT_CTRL_4;
    case static_cast<uint32_t>(CanIdPiper::ARM_JOINT_MIT_CTRL_5):
      return MessageType::JOINT_MIT_CTRL_5;
    case static_cast<uint32_t>(CanIdPiper::ARM_JOINT_MIT_CTRL_6):
      return MessageType::JOINT_MIT_CTRL_6;
    // Joint velocity + acceleration feedback
    case static_cast<uint32_t>(CanIdPiper::ARM_FEEDBACK_JOINT_VEL_ACC_1):
      return MessageType::JOINT_VEL_ACC_1;
    case static_cast<uint32_t>(CanIdPiper::ARM_FEEDBACK_JOINT_VEL_ACC_2):
      return MessageType::JOINT_VEL_ACC_2;
    case static_cast<uint32_t>(CanIdPiper::ARM_FEEDBACK_JOINT_VEL_ACC_3):
      return MessageType::JOINT_VEL_ACC_3;
    case static_cast<uint32_t>(CanIdPiper::ARM_FEEDBACK_JOINT_VEL_ACC_4):
      return MessageType::JOINT_VEL_ACC_4;
    case static_cast<uint32_t>(CanIdPiper::ARM_FEEDBACK_JOINT_VEL_ACC_5):
      return MessageType::JOINT_VEL_ACC_5;
    case static_cast<uint32_t>(CanIdPiper::ARM_FEEDBACK_JOINT_VEL_ACC_6):
      return MessageType::JOINT_VEL_ACC_6;
    default:
      return MessageType::UNKNOWN;
  }
}

}  // namespace piper
}  // namespace agilex