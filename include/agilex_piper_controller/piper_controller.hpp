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
#pragma once

#include <array>
#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "agilex_piper_controller/can_interface.hpp"
#include "agilex_piper_controller/piper_protocol.hpp"
#include "agilex_piper_controller/piper_types.hpp"

namespace agilex
{
namespace piper
{

/**
 * Main controller class for Piper robot
 *
 * This class provides a high-level interface to control the Piper robot.
 */
class PiperController
{
public:
  /**
   * Constructor
   *
   * Parameters:
   *   can_interface_name - Name of the CAN interface
   *   auto_init - Whether to automatically initialize the controller
   *   dh_is_offset - Whether DH parameters are offset (1) or standard (0)
   *   enable_sdk_joint_limits - Whether to enable SDK joint limits
   *   enable_sdk_gripper_limits - Whether to enable SDK gripper limits
   */
  PiperController(
    const std::string & can_interface_name, bool auto_init = true, int dh_is_offset = 0,
    bool enable_sdk_joint_limits = true, bool enable_sdk_gripper_limits = true);

  /**
   * Destructor
   */
  ~PiperController();

  /**
   * Connect to the CAN port
   *
   * Parameters:
   *   init_can - Whether to initialize the CAN interface
   *
   * Returns true if successful, false otherwise
   */
  bool connect_port(bool init_can = true);

  /**
   * Disconnect from the CAN port
   */
  void disconnect();

  /**
   * Check if the controller is connected
   *
   * Returns true if connected, false otherwise
   */
  bool is_connected() const;

  /**
   * Get arm status
   *
   * Returns arm status
   */
  ArmStatus get_arm_status() const;

  /**
   * Get arm end pose
   *
   * Returns arm end pose
   */
  ArmEndPose get_arm_end_pose() const;

  /**
   * Get arm joint angles
   *
   * Returns arm joint angles
   */
  ArmJoint get_arm_joint() const;

  /**
   * Get arm gripper state
   *
   * Returns arm gripper state
   */
  ArmGripper get_arm_gripper() const;

  /**
   * Enable the arm
   *
   * Returns true if successful, false otherwise
   */
  bool enable_arm();

  /**
   * Disable the arm
   *
   * Returns true if successful, false otherwise
   */
  bool disable_arm();

  /**
   * Set control mode
   *
   * Parameters:
   *   ctrl_mode - Control mode (0x00: Standby, 0x01: CAN command)
   *   move_mode - Move mode (0x00: Position, 1: Joint, 4: MIT)
   *   move_speed_rate - Movement speed rate (0-100)
   *   is_mit_mode - 0x00: Position-speed mode, 0xAD: MIT mode, 0xFF: Invalid
   *
   * Returns true if successful, false otherwise
   */
  bool set_mode(
    uint8_t ctrl_mode, uint8_t move_mode, uint8_t move_speed_rate, uint8_t is_mit_mode = 0);

  /**
   * Set end pose
   *
   * Parameters:
   *   x - X position (0.001mm)
   *   y - Y position (0.001mm)
   *   z - Z position (0.001mm)
   *   rx - RX rotation (0.001 degrees)
   *   ry - RY rotation (0.001 degrees)
   *   rz - RZ rotation (0.001 degrees)
   *
   * Returns true if successful, false otherwise
   */
  bool set_end_pose(int x, int y, int z, int rx, int ry, int rz);

  /**
   * Set joint angles
   *
   * Parameters:
   *   j1 - Joint 1 angle (0.001 degrees)
   *   j2 - Joint 2 angle (0.001 degrees)
   *   j3 - Joint 3 angle (0.001 degrees)
   *   j4 - Joint 4 angle (0.001 degrees)
   *   j5 - Joint 5 angle (0.001 degrees)
   *   j6 - Joint 6 angle (0.001 degrees)
   *
   * Returns true if successful, false otherwise
   */
  bool set_joint_angles(int j1, int j2, int j3, int j4, int j5, int j6);

  /**
   * Control the gripper
   *
   * Parameters:
   *   grippers_angle - Gripper angle in 0.001° units (mm)
   *   grippers_effort - Gripper torque in 0.001N·m units (range: 0-5000)
   *   status_code - Status code for enable/disable/clear error:
   *                 0x00: Disable
   *                 0x01: Enable
   *                 0x02: Disable with clear error
   *                 0x03: Enable with clear error
   *   set_zero - Set current position as zero point:
   *              0x00: Invalid
   *              0xAE: Set zero
   *
   * Returns true if successful, false otherwise
   */
  bool control_gripper(
    int grippers_angle, uint16_t grippers_effort = 1000, uint8_t status_code = 0x01,
    uint8_t set_zero = 0x00);

  /**
   * Update C-axis movement
   *
   * Parameters:
   *   instruction_num - Instruction number
   *
   * Returns true if successful, false otherwise
   */
  bool move_c_axis_update(uint8_t instruction_num);

  /**
   * Configure a joint
   *
   * Parameters:
   *   joint_id - Joint ID
   *   enable_pos_lim - Enable position limit
   *   enable_vel_lim - Enable velocity limit
   *   max_joint_acc - Maximum joint acceleration
   *
   * Returns true if successful, false otherwise
   */
  bool configure_joint(
    uint8_t joint_id, uint8_t enable_pos_lim, uint8_t enable_vel_lim, int max_joint_acc);

  /**
   * Configure crash protection
   *
   * Parameters:
   *   j1_level - Joint 1 protection level
   *   j2_level - Joint 2 protection level
   *   j3_level - Joint 3 protection level
   *   j4_level - Joint 4 protection level
   *   j5_level - Joint 5 protection level
   *   j6_level - Joint 6 protection level
   *
   * Returns true if successful, false otherwise
   */
  bool configure_crash_protection(
    uint8_t j1_level, uint8_t j2_level, uint8_t j3_level, uint8_t j4_level, uint8_t j5_level,
    uint8_t j6_level);

  /**
   * Configure master-slave mode
   *
   * Parameters:
   *   master_slave_mode - Master-slave mode
   *   teach_mode - Teach mode
   *   user_value1 - User value 1
   *   user_value2 - User value 2
   *
   * Returns true if successful, false otherwise
   */
  bool set_master_slave_config(
    uint8_t master_slave_mode, uint8_t teach_mode, uint8_t user_value1 = 0,
    uint8_t user_value2 = 0);

  /**
   * Set motion control type 1
   *
   * Parameters:
   *   emergency_stop - Emergency stop control (0x00: Invalid, 0x01: Activate emergency stop, 0x02: Resume)
   *   track_ctrl - Trajectory control instructions
   *   grag_teach_ctrl - Drag teach control
   *   trajectory_index - Trajectory point index (0-255)
   *   name_index - Trajectory packet name index
   *
   * Returns true if successful, false otherwise
   */
  bool motion_control_1(
    uint8_t emergency_stop, uint8_t track_ctrl = 0, uint8_t grag_teach_ctrl = 0,
    uint8_t trajectory_index = 0, uint16_t name_index = 0);

  /**
   * Set motion control type 2
   *
   * Parameters:
   *   ctrl_mode - Control mode
   *   move_mode - Move mode
   *   move_speed_rate - Movement speed rate
   *   is_mit_mode - MIT mode flag
   *   residence_time - Residence time
   *   installation_pos - Installation position
   *
   * Returns true if successful, false otherwise
   */
  bool motion_control_2(
    uint8_t ctrl_mode, uint8_t move_mode, uint8_t move_speed_rate, uint8_t is_mit_mode = 0,
    uint8_t residence_time = 0, uint8_t installation_pos = 0);

  /**
   * Get firmware version
   *
   * Returns firmware version string
   */
  std::string get_firmware_version();

  /**
   * Get SDK joint limit parameters
   *
   * Parameters:
   *   joint_name - Joint name (j1, j2, etc.)
   *
   * Returns pair of minimum and maximum joint angle (radians)
   */
  [[nodiscard]] std::pair<double, double> get_sdk_joint_limit_param(const std::string & joint_name);

  /**
   * Get SDK gripper range parameters
   *
   * Returns pair of minimum and maximum gripper position
   */
  [[nodiscard]] std::pair<double, double> get_sdk_gripper_range_param();

  /**
   * Set SDK joint limit parameters
   *
   * Parameters:
   *   joint_name - Joint name (j1, j2, etc.)
   *   min_val - Minimum joint angle (radians)
   *   max_val - Maximum joint angle (radians)
   */
  void set_sdk_joint_limit_param(const std::string & joint_name, double min_val, double max_val);

  /**
   * Set SDK gripper range parameters
   *
   * Parameters:
   *   min_val - Minimum gripper position
   *   max_val - Maximum gripper position
   */
  void set_sdk_gripper_range_param(double min_val, double max_val);

  /**
   * Get motor information for a specific joint
   *
   * Parameters:
   *   joint_num - Joint number (1-6)
   *
   * Returns motor information for the specified joint
   */
  MotorInfo get_motor_info(uint8_t joint_num) const;

private:
  std::string can_interface_name_;  // CAN interface name
  bool auto_init_;                  // Whether to automatically initialize
  int dh_is_offset_;                // Whether DH parameters are offset
  bool enable_sdk_joint_limits_;    // Whether to enable SDK joint limits
  bool enable_sdk_gripper_limits_;  // Whether to enable SDK gripper limits

  std::unique_ptr<PiperParams> parameters_;      // Parameters
  std::unique_ptr<PiperProtocolBase> protocol_;  // Protocol - changed from PiperProtocol
  std::unique_ptr<CanInterface> can_interface_;  // CAN interface

  std::atomic<bool> running_;       // Whether the controller is running
  std::thread read_can_thread_;     // Thread for reading CAN frames
  std::thread can_monitor_thread_;  // Thread for monitoring CAN connection

  mutable std::mutex arm_status_mutex_;    // Mutex for arm status
  mutable std::mutex arm_end_pose_mutex_;  // Mutex for arm end pose
  mutable std::mutex arm_joint_mutex_;     // Mutex for arm joint angles
  mutable std::mutex arm_gripper_mutex_;   // Mutex for arm gripper state
  mutable std::mutex firmware_mutex_;      // Mutex for firmware data
  mutable std::mutex motor_info_mutex_;    // Mutex for motor information

  ArmStatus arm_status_;                 // Arm status
  ArmEndPose arm_end_pose_;              // Arm end pose
  ArmJoint arm_joint_;                   // Arm joint angles
  ArmGripper arm_gripper_;               // Arm gripper state
  std::vector<uint8_t> firmware_data_;   // Firmware data
  std::array<MotorInfo, 6> motor_info_;  // Motor information for joints 1-6

  void parse_can_frame(CanFrameMsg & frame);  // Parse CAN frame

  void read_can_loop();     // Thread function for reading CAN frames
  void can_monitor_loop();  // Thread function for monitoring CAN connection

  // Helper methods for sending commands
  bool cartesian_ctrl_xy(int x, int y);
  bool cartesian_ctrl_zrx(int z, int rx);
  bool cartesian_ctrl_ryrz(int ry, int rz);
  bool joint_ctrl_12(int j1, int j2);
  bool joint_ctrl_34(int j3, int j4);
  bool joint_ctrl_56(int j5, int j6);

  // Helper method to check joint limits
  int check_joint_sdk_limit(int joint_value, const std::string & joint_name);
};

}  // namespace piper
}  // namespace agilex