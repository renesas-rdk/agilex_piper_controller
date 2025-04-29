#pragma once

#include <cstdint>
#include <variant>
#include <array>
#include <string>

namespace agilex
{
namespace piper
{

/**
 * CAN IDs for Piper messages
 */
enum class CanIdPiper : uint32_t
{
  // Control messages (match IDs in piper_sdk/piper_msgs/msg_v2/can_id.py)
  ARM_MOTION_CTRL_1 = 0x150,
  ARM_MOTION_CTRL_2 = 0x151,
  ARM_MOTION_CTRL_CARTESIAN_1 = 0x152,
  ARM_MOTION_CTRL_CARTESIAN_2 = 0x153,
  ARM_MOTION_CTRL_CARTESIAN_3 = 0x154,
  ARM_JOINT_CTRL_12 = 0x155,
  ARM_JOINT_CTRL_34 = 0x156,
  ARM_JOINT_CTRL_56 = 0x157,
  ARM_CIRCULAR_PATTERN_COORD_NUM_UPDATE_CTRL = 0x158,
  ARM_GRIPPER_CTRL = 0x159,
  // MIT control (V1.5-2 and later)
  ARM_JOINT_MIT_CTRL_1 = 0x15A,
  ARM_JOINT_MIT_CTRL_2 = 0x15B,
  ARM_JOINT_MIT_CTRL_3 = 0x15C,
  ARM_JOINT_MIT_CTRL_4 = 0x15D,
  ARM_JOINT_MIT_CTRL_5 = 0x15E,
  ARM_JOINT_MIT_CTRL_6 = 0x15F,

  // Parameter configuration commands
  ARM_MASTER_SLAVE_MODE_CONFIG = 0x470,
  ARM_MOTOR_ENABLE_DISABLE_CONFIG = 0x471,
  ARM_SEARCH_MOTOR_MAX_SPD_ACC_LIMIT = 0x472,
  ARM_FEEDBACK_CURRENT_MOTOR_ANGLE_LIMIT_MAX_SPD = 0x473,
  ARM_MOTOR_ANGLE_LIMIT_MAX_SPD_SET = 0x474,
  ARM_JOINT_CONFIG = 0x475,
  ARM_INSTRUCTION_RESPONSE_CONFIG = 0x476,
  ARM_PARAM_ENQUIRY_AND_CONFIG = 0x477,
  ARM_FEEDBACK_CURRENT_END_VEL_ACC_PARAM = 0x478,
  ARM_END_VEL_ACC_PARAM_CONFIG = 0x479,
  ARM_CRASH_PROTECTION_RATING_CONFIG = 0x47A,
  ARM_CRASH_PROTECTION_RATING_FEEDBACK = 0x47B,
  ARM_FEEDBACK_CURRENT_MOTOR_MAX_ACC_LIMIT = 0x47C,
  ARM_GRIPPER_TEACHING_PENDANT_PARAM_CONFIG = 0x47D,
  ARM_GRIPPER_TEACHING_PENDANT_PARAM_FEEDBACK = 0x47E,

  // Joint velocity/acceleration feedback
  ARM_FEEDBACK_JOINT_VEL_ACC_1 = 0x481,
  ARM_FEEDBACK_JOINT_VEL_ACC_2 = 0x482,
  ARM_FEEDBACK_JOINT_VEL_ACC_3 = 0x483,
  ARM_FEEDBACK_JOINT_VEL_ACC_4 = 0x484,
  ARM_FEEDBACK_JOINT_VEL_ACC_5 = 0x485,
  ARM_FEEDBACK_JOINT_VEL_ACC_6 = 0x486,

  // Light control
  ARM_LIGHT_CTRL = 0x121,

  // Motor info (high speed)
  ARM_INFO_HIGH_SPD_FEEDBACK_1 = 0x251,
  ARM_INFO_HIGH_SPD_FEEDBACK_2 = 0x252,
  ARM_INFO_HIGH_SPD_FEEDBACK_3 = 0x253,
  ARM_INFO_HIGH_SPD_FEEDBACK_4 = 0x254,
  ARM_INFO_HIGH_SPD_FEEDBACK_5 = 0x255,
  ARM_INFO_HIGH_SPD_FEEDBACK_6 = 0x256,

  // Motor info (low speed)
  ARM_INFO_LOW_SPD_FEEDBACK_1 = 0x261,
  ARM_INFO_LOW_SPD_FEEDBACK_2 = 0x262,
  ARM_INFO_LOW_SPD_FEEDBACK_3 = 0x263,
  ARM_INFO_LOW_SPD_FEEDBACK_4 = 0x264,
  ARM_INFO_LOW_SPD_FEEDBACK_5 = 0x265,
  ARM_INFO_LOW_SPD_FEEDBACK_6 = 0x266,

  // Feedback messages
  ARM_STATUS_FEEDBACK = 0x2A1,
  ARM_END_POSE_XY_FEEDBACK = 0x2A2,
  ARM_END_POSE_ZRX_FEEDBACK = 0x2A3,
  ARM_END_POSE_RYRZ_FEEDBACK = 0x2A4,
  ARM_JOINT_FEEDBACK_12 = 0x2A5,
  ARM_JOINT_FEEDBACK_34 = 0x2A6,
  ARM_JOINT_FEEDBACK_56 = 0x2A7,
  ARM_GRIPPER_FEEDBACK = 0x2A8,

  // System commands
  ARM_CAN_UPDATE_SILENT_MODE_CONFIG = 0x422,
  ARM_FIRMWARE_READ = 0x4AF,
};

/**
 * Message types for Piper
 */
enum class MessageType
{
  UNKNOWN,

  // Control message types
  ENABLE_DISABLE_ARM,
  MOTION_CTRL_1,
  MOTION_CTRL_2,
  CARTESIAN_CTRL_1,
  CARTESIAN_CTRL_2,
  CARTESIAN_CTRL_3,
  JOINT_CTRL_12,
  JOINT_CTRL_34,
  JOINT_CTRL_56,
  GRIPPER_CTRL,
  JOINT_CONFIG,
  CRASH_PROTECTION_CONFIG,
  MASTER_SLAVE_CONFIG,
  CIRCULAR_PATTERN_COORD_UPDATE,
  PARAM_ENQUIRY_CONFIG,
  LIGHT_CTRL,

  // Feedback message types
  ARM_STATUS,
  END_POSE_XY,
  END_POSE_ZRX,
  END_POSE_RYRZ,
  JOINT_12,
  JOINT_34,
  JOINT_56,
  GRIPPER,

  // Generic motor info types
  MOTOR_INFO_HIGH_SPD,
  MOTOR_INFO_LOW_SPD,

  FIRMWARE_VERSION,
};

// Message structure definitions

/**
 * Enable/Disable arm message
 *
 * CAN ID: 0x471
 */
struct MsgEnableDisableArm
{
  uint8_t motor_num;    // Motor index: 1-6: joint motors, 7: gripper motor, 0xFF: all motors
  uint8_t enable_flag;  // 0x01: Disable, 0x02: Enable
  uint8_t reserved1;
  uint8_t reserved2;
  uint8_t reserved3;
  uint8_t reserved4;
  uint8_t reserved5;
  uint8_t reserved6;
};

/**
 * Motion control 1 message
 *
 * CAN ID: 0x150
 *
 * Robotic Arm Motion Control Command 1
 */
struct MsgMotionCtrl1
{
  uint8_t emergency_stop;    // Emergency stop control:
                             // 0x00: Invalid
                             // 0x01: Activate emergency stop
                             // 0x02: Resume from emergency stop
  uint8_t track_ctrl;        // Trajectory control instructions:
                             // 0x00: Disable
                             // 0x01: Pause current planning
                             // 0x02: Resume current trajectory
                             // 0x03: Clear current trajectory
                             // 0x04: Clear all trajectories
                             // 0x05: Get current planned trajectory
                             // 0x06: Terminate execution
                             // 0x07: Trajectory transfer
                             // 0x08: End trajectory transfer
  uint8_t grag_teach_ctrl;   // Drag teach control:
                             // 0x00: Disable
                             // 0x01: Start teaching record (enter drag teach mode)
                             // 0x02: End teaching record (exit drag teach mode)
                             // 0x03: Execute teaching trajectory
                             // 0x04: Pause execution
                             // 0x05: Continue execution
                             // 0x06: Terminate execution
                             // 0x07: Move to trajectory starting point
  uint8_t trajectory_index;  // Trajectory point index (0-255)
                             // Controller responds with CAN ID: 0x476
  uint16_t name_index;       // Trajectory packet name index
  uint16_t crc16;            // CRC checksum for validation
};

/**
 * Motion control 2 message
 *
 * CAN ID: 0x151
 *
 * Robotic Arm Motion Control Command 2
 */
struct MsgMotionCtrl2
{
  uint8_t ctrl_mode;         // Control mode:
                             // 0x00: Standby mode
                             // 0x01: CAN command control mode
                             // 0x03: Ethernet control mode
                             // 0x04: Wi-Fi control mode
                             // 0x07: Offline trajectory mode
  uint8_t move_mode;         // Movement mode:
                             // 0x00: MOVE P (Position)
                             // 0x01: MOVE J (Joint)
                             // 0x02: MOVE L (Linear)
                             // 0x03: MOVE C (Circular)
                             // 0x04: MOVE M (MIT) - Based on version V1.5-2 and later
  uint8_t move_speed_rate;   // Movement speed as a percentage (0-100)
  uint8_t is_mit_mode;       // MIT mode:
                             // 0x00: Position-speed mode
                             // 0xAD: MIT mode
                             // 0xFF: Invalid
  uint8_t residence_time;    // Hold time at offline trajectory points (0-255 seconds)
                             // 255: Trajectory termination
  uint8_t installation_pos;  // Installation position (wiring facing backward):
                             // 0x00: Invalid value
                             // 0x01: Horizontal upright
                             // 0x02: Left-side mount
                             // 0x03: Right-side mount
  uint8_t reserved1;
  uint8_t reserved2;
};

/**
 * Cartesian control message
 */
struct MsgCartesianCtrl
{
  int32_t axis1;  // X, Z, or RY axis
  int32_t axis2;  // Y, RX, or RZ axis
};

/**
 * Joint control 12 message
 */
struct MsgJointCtrl12
{
  int32_t joint_1;
  int32_t joint_2;
};

/**
 * Joint control 34 message
 */
struct MsgJointCtrl34
{
  int32_t joint_3;
  int32_t joint_4;
};

/**
 * Joint control 56 message
 */
struct MsgJointCtrl56
{
  int32_t joint_5;
  int32_t joint_6;
};

/**
 * Gripper control message
 *
 * CAN ID: 0x159
 */
struct MsgGripperCtrl
{
  int32_t grippers_angle;    // Gripper angle in 0.001° units
  uint16_t grippers_effort;  // Gripper torque in 0.001N·m units (range: 0-5000)
  uint8_t status_code;       // Status code for enable/disable/clear error:
                             // 0x00: Disable
                             // 0x01: Enable
                             // 0x02: Disable with clear error
                             // 0x03: Enable with clear error
  uint8_t set_zero;          // Set current position as zero point:
                             // 0x00: Invalid
                             // 0xAE: Set zero
};

/**
 * Joint configuration message
 */
struct MsgJointConfig
{
  uint8_t joint_num;               // Joint number
  uint8_t set_zero;                // Set zero position
  uint8_t acc_param_is_effective;  // Acceleration parameter effective
  uint16_t max_joint_acc;          // Maximum joint acceleration
  uint8_t clear_joint_err;         // Clear joint error
};

/**
 * Crash protection configuration message
 */
struct MsgCrashProtectionConfig
{
  uint8_t j1_level;  // Joint 1 protection level
  uint8_t j2_level;  // Joint 2 protection level
  uint8_t j3_level;  // Joint 3 protection level
  uint8_t j4_level;  // Joint 4 protection level
  uint8_t j5_level;  // Joint 5 protection level
  uint8_t j6_level;  // Joint 6 protection level
  uint8_t reserved1;
  uint8_t reserved2;
};

/**
 * Master/slave configuration message
 */
struct MsgMasterSlaveConfig
{
  uint8_t master_slave_mode;  // Master/slave mode
  uint8_t teach_mode;         // Teaching mode
  uint8_t user_value1;        // User value 1
  uint8_t user_value2;        // User value 2
  uint8_t reserved1;
  uint8_t reserved2;
  uint8_t reserved3;
  uint8_t reserved4;
};

/**
 * Circular pattern coordinate update message
 */
struct MsgCircularPatternCoordUpdate
{
  uint8_t instruction_num;  // Instruction number
  uint8_t reserved1;
  uint8_t reserved2;
  uint8_t reserved3;
  uint8_t reserved4;
  uint8_t reserved5;
  uint8_t reserved6;
  uint8_t reserved7;
};

/**
 * Parameter enquiry and configuration message
 */
struct MsgParamEnquiryConfig
{
  uint8_t param_type;  // Parameter type
  uint8_t operation;   // Operation
  uint8_t reserved1;
  uint8_t reserved2;
  uint32_t param_value;  // Parameter value
};

/**
 * Light control message
 */
struct MsgLightCtrl
{
  uint8_t light_ctrl;  // Light control
  uint8_t light_mode;  // Light mode
  uint8_t light_r;     // Red component
  uint8_t light_g;     // Green component
  uint8_t light_b;     // Blue component
  uint8_t reserved1;
  uint8_t reserved2;
  uint8_t reserved3;
};

/**
 * Arm status feedback message
 *
 * CAN ID: 0x2A1
 */
struct MsgArmStatusFeedback
{
  uint8_t ctrl_mode;  // Control mode:
                      // 0x00: Standby mode
                      // 0x01: CAN instruction control mode
                      // 0x02: Teaching mode
                      // 0x03: Ethernet control mode
                      // 0x04: Wi-Fi control mode
                      // 0x05: Remote control mode
                      // 0x06: Linkage teaching input mode
                      // 0x07: Offline trajectory mode

  uint8_t arm_status;  // Robot arm status:
                       // 0x00: Normal
                       // 0x01: Emergency stop
                       // 0x02: No solution
                       // 0x03: Singularity point
                       // 0x04: Target angle exceeds limit
                       // 0x05: Joint communication exception
                       // 0x06: Joint brake not released
                       // 0x07: Collision occurred
                       // 0x08: Overspeed during teaching drag
                       // 0x09: Joint status abnormal
                       // 0x0A: Other exception
                       // 0x0B: Teaching record
                       // 0x0C: Teaching execution
                       // 0x0D: Teaching pause
                       // 0x0E: Main controller NTC over temperature
                       // 0x0F: Release resistor NTC over temperature

  uint8_t mode_feed;  // Mode feedback:
                      // 0x00: MOVE P
                      // 0x01: MOVE J
                      // 0x02: MOVE L
                      // 0x03: MOVE C
                      // 0x04: MOVE M (V1.5-2 and later)

  uint8_t teach_status;  // Teaching status:
                         // 0x00: Off
                         // 0x01: Start teaching record (enter drag teaching mode)
                         // 0x02: End teaching record (exit drag teaching mode)
                         // 0x03: Execute teaching trajectory
                         // 0x04: Pause execution
                         // 0x05: Continue execution
                         // 0x06: Terminate execution
                         // 0x07: Move to trajectory starting point

  uint8_t motion_status;  // Motion status:
                          // 0x00: Reached the target position
                          // 0x01: Not yet reached the target position

  uint8_t trajectory_num;  // Current trajectory point number (0-255)
                           // Used in offline trajectory mode

  uint8_t err_code_comm;  // Error code - communication status:
                          // bit[0]: Joint 1 communication exception (0: normal, 1: abnormal)
                          // bit[1]: Joint 2 communication exception (0: normal, 1: abnormal)
                          // bit[2]: Joint 3 communication exception (0: normal, 1: abnormal)
                          // bit[3]: Joint 4 communication exception (0: normal, 1: abnormal)
                          // bit[4]: Joint 5 communication exception (0: normal, 1: abnormal)
                          // bit[5]: Joint 6 communication exception (0: normal, 1: abnormal)
                          // bit[6-7]: Reserved

  uint8_t err_code_angle;  // Error code - angle limits:
                           // bit[0]: Joint 1 angle limit exceeded (0: normal, 1: abnormal)
                           // bit[1]: Joint 2 angle limit exceeded (0: normal, 1: abnormal)
                           // bit[2]: Joint 3 angle limit exceeded (0: normal, 1: abnormal)
                           // bit[3]: Joint 4 angle limit exceeded (0: normal, 1: abnormal)
                           // bit[4]: Joint 5 angle limit exceeded (0: normal, 1: abnormal)
                           // bit[5]: Joint 6 angle limit exceeded (0: normal, 1: abnormal)
                           // bit[6-7]: Reserved
};

/**
 * Firmware version message
 */
struct MsgFirmwareVersion
{
  // The firmware version is stored as a string of up to 8 characters
  uint8_t version_data[8];
};

/**
 * High-Speed Feedback of Drive Information
 *
 * CAN ID: 0x251~0x256 (joints 1-6)
 */
struct MsgArmHighSpeedFeedback
{
  int16_t motor_speed;  // Motor speed (0.001rad/s)
  uint16_t current;     // Motor current (0.001A)
  int32_t position;     // Motor position (rad)
};

/**
 * Low-Speed Feedback of Drive Information
 *
 * CAN ID: 0x261~0x266 (joints 1-6)
 */
struct MsgArmLowSpeedFeedback
{
  uint16_t voltage;           // Bus voltage (0.1V)
  int16_t drive_temperature;  // Drive temperature (°C)
  int8_t motor_temperature;   // Motor temperature (°C)
  uint8_t drive_status;       // Drive status flags:
                              // bit[0]: Power voltage low (0: Normal, 1: Low)
                              // bit[1]: Motor over-temperature (0: Normal, 1: Over-temperature)
                              // bit[2]: Drive over-current (0: Normal, 1: Over-current)
                              // bit[3]: Drive over-temperature (0: Normal, 1: Over-temperature)
                              // bit[4]: Collision protection status (0: Normal, 1: Trigger protection)
                              // bit[5]: Drive error status (0: Normal, 1: Error)
                              // bit[6]: Drive enable status (1: Enabled, 0: Disabled)
                              // bit[7]: Stalling protection status (0: Normal, 1: Trigger protection)
  uint16_t bus_current;       // Bus current (0.001A)
};

/**
 * Piper message variant
 *
 * Uses std::variant to hold different message types
 */
using PiperMessageData =
    std::variant<std::monostate,  // Empty state
                 MsgEnableDisableArm, MsgMotionCtrl1, MsgMotionCtrl2, MsgCartesianCtrl, MsgJointCtrl12, MsgJointCtrl34,
                 MsgJointCtrl56, MsgGripperCtrl, MsgJointConfig, MsgCrashProtectionConfig, MsgMasterSlaveConfig,
                 MsgCircularPatternCoordUpdate, MsgParamEnquiryConfig, MsgLightCtrl, MsgArmStatusFeedback,
                 MsgFirmwareVersion, MsgArmHighSpeedFeedback, MsgArmLowSpeedFeedback>;

/**
 * Piper message class
 *
 * Contains a message type and the associated data
 */
class PiperMessage
{
public:
  /**
   * Constructor
   *
   * Parameters:
   *   type - Message type
   */
  PiperMessage(MessageType type = MessageType::UNKNOWN);

  /**
   * Constructor with data
   *
   * Parameters:
   *   type - Message type
   *   data - Message data
   */
  PiperMessage(MessageType type, const PiperMessageData& data);

  /**
   * Get the message type
   *
   * Returns message type
   */
  MessageType get_type() const
  {
    return type_;
  }

  /**
   * Set the message type
   *
   * Parameters:
   *   type - Message type
   */
  void set_type(MessageType type)
  {
    type_ = type;
  }

  /**
   * Get the message data
   *
   * Returns message data
   */
  const PiperMessageData& get_data() const
  {
    return data_;
  }

  /**
   * Set the message data
   *
   * Parameters:
   *   data - Message data
   */
  void set_data(const PiperMessageData& data)
  {
    data_ = data;
  }

  /**
   * Convert message type to CAN ID
   *
   * Parameters:
   *   type - Message type
   *
   * Returns CAN ID
   */
  static CanIdPiper message_type_to_can_id(MessageType type);

  /**
   * Convert CAN ID to message type
   *
   * Parameters:
   *   can_id - CAN ID
   *
   * Returns message type
   */
  static MessageType can_id_to_message_type(uint32_t can_id);

private:
  MessageType type_;       ///< Message type
  PiperMessageData data_;  ///< Message data
};

}  // namespace piper
}  // namespace agilex