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
 * Main controller class for the Agilex Piper robot arm.
 *
 * Provides two layers of API:
 *
 *   1. Raw / position-mode API — joint angles in 0.001-degree protocol units,
 *      matching the original Python piper_sdk interface.
 *
 *   2. SI-unit API — joint angles in radians, full RobotState snapshots, and
 *      per-joint MIT-mode commands (position + velocity + torque feedforward).
 *      Inspired by libfranka and arx5-sdk.
 *
 * CAN frames are received via a callback in the CanInterface read thread and
 * decoded directly into the internal state protected by per-field mutexes.
 */
class PiperController
{
public:
  /**
   * Constructor
   *
   * Parameters:
   *   can_interface_name       - Name of the CAN interface (e.g. "can0")
   *   auto_init                - Connect and start threads immediately
   *   dh_is_offset             - DH convention flag (0: standard, 1: offset)
   *   enable_sdk_joint_limits  - Clamp joint commands to SDK soft limits
   *   enable_sdk_gripper_limits- Clamp gripper command to SDK soft limits
   *   mit_cfg                  - Fixed-point encoding limits for MIT mode
   */
  explicit PiperController(
    const std::string & can_interface_name, bool auto_init = true, int dh_is_offset = 0,
    bool enable_sdk_joint_limits = true, bool enable_sdk_gripper_limits = true,
    const MitConfig & mit_cfg = MitConfig{});

  ~PiperController();

  // ── Connection ─────────────────────────────────────────────────────────────

  /** Open socket, configure CAN interface, start receive thread. */
  bool connect_port(bool init_can = true);

  /** Stop threads and close socket. */
  void disconnect();

  /** Returns true while the CAN socket is open and the monitor thread is alive. */
  bool is_connected() const;

  // ── Raw state getters (protocol units) ─────────────────────────────────────

  ArmStatus get_arm_status() const;
  ArmEndPose get_arm_end_pose() const;
  ArmJoint get_arm_joint() const;
  ArmGripper get_arm_gripper() const;

  /**
   * Get motor information for a specific joint (1-indexed).
   *
   * Parameters:
   *   joint_num - Joint number (1–6)
   */
  MotorInfo get_motor_info(uint8_t joint_num) const;

  // ── SI-unit state API ───────────────────────────────────────────────────────

  /**
   * Return a consistent snapshot of the full robot state in SI units.
   *
   * Joint positions [rad] come from the joint-angle feedback (0x2A5–0x2A7).
   * Joint velocities [rad/s] come from the joint vel/acc feedback (0x481–0x486)
   * or fall back to the high-speed motor feedback (0x251–0x256).
   * Joint current [A] comes from the high-speed motor feedback.
   */
  RobotState get_robot_state() const;

  // ── Arm enable / disable ───────────────────────────────────────────────────

  /**
   * Enable all motors (joints 1–6 + gripper).
   * Retries up to 20 times at 10 ms intervals until all drives report "enabled".
   */
  bool enable_arm();

  /**
   * Disable all motors.
   */
  bool disable_arm();

  // ── Mode selection ─────────────────────────────────────────────────────────

  /**
   * Set control / move mode (wraps motion_control_2).
   *
   * Parameters:
   *   ctrl_mode       - 0x00: standby, 0x01: CAN command control
   *   move_mode       - 0x00: MOVE P, 0x01: MOVE J, 0x04: MOVE M (MIT)
   *   move_speed_rate - Speed as percent (0–100); ignored in MIT mode
   *   is_mit_mode     - 0x00: position-speed, 0xAD: MIT, 0xFF: invalid
   */
  bool set_mode(
    uint8_t ctrl_mode, uint8_t move_mode, uint8_t move_speed_rate, uint8_t is_mit_mode = 0);

  /**
   * Convenience: switch to MIT (torque) mode.
   *
   * Sends motion_control_2 with move_mode=0x04 and is_mit_mode=0xAD.
   */
  bool enable_mit_mode();

  /**
   * Convenience: switch back to joint position mode (MOVE J).
   */
  bool enable_position_mode(uint8_t speed_rate = 50);

  // ── Position-mode motion (raw protocol units) ──────────────────────────────

  /**
   * Set Cartesian end-effector pose.
   *
   * Parameters: x, y, z [0.001 mm], rx, ry, rz [0.001 deg]
   */
  bool set_end_pose(int x, int y, int z, int rx, int ry, int rz);

  /**
   * Set joint angles in protocol units (0.001 degrees).
   *
   * SDK soft limits are applied when enable_sdk_joint_limits is true.
   */
  bool set_joint_angles(int j1, int j2, int j3, int j4, int j5, int j6);

  // ── SI-unit motion ─────────────────────────────────────────────────────────

  /**
   * Set all joint angles in radians.
   *
   * Converts to protocol units and calls set_joint_angles().
   * SDK soft limits are applied.
   */
  bool set_joint_angles_rad(const std::array<double, 6> & q_rad);

  /**
   * Send a single-joint MIT command.
   *
   * joint_idx is 0-based (0 = J1 … 5 = J6).
   *
   * The robot must be in MIT mode (enable_mit_mode()) before calling this.
   * The internal MitConfig provides the fixed-point encoding ranges.
   */
  bool send_mit_cmd(int joint_idx, const MitJointCommand & cmd);

  /**
   * Send MIT commands to all six joints atomically (three CAN frames).
   *
   * This is the primary interface for model-based / torque-control loops.
   */
  bool send_mit_cmd_all(const std::array<MitJointCommand, 6> & cmds);

  // ── Gripper ────────────────────────────────────────────────────────────────

  /**
   * Control the gripper.
   *
   * Parameters:
   *   grippers_angle  - Position [0.001 deg]
   *   grippers_effort - Torque limit [0.001 Nm], range 0–5000
   *   status_code     - 0x00 disable, 0x01 enable, 0x02 disable+clear, 0x03 enable+clear
   *   set_zero        - 0x00 no-op, 0xAE set current position as zero
   */
  bool control_gripper(
    int grippers_angle, uint16_t grippers_effort = 1000, uint8_t status_code = 0x01,
    uint8_t set_zero = 0x00);

  // ── Configuration commands ─────────────────────────────────────────────────

  bool move_c_axis_update(uint8_t instruction_num);

  bool configure_joint(
    uint8_t joint_id, uint8_t set_zero, uint8_t acc_param_effective, int max_joint_acc,
    uint8_t clear_joint_err = 0);

  bool configure_crash_protection(
    uint8_t j1_level, uint8_t j2_level, uint8_t j3_level, uint8_t j4_level, uint8_t j5_level,
    uint8_t j6_level);

  bool set_master_slave_config(
    uint8_t master_slave_mode, uint8_t teach_mode, uint8_t user_value1 = 0,
    uint8_t user_value2 = 0);

  bool motion_control_1(
    uint8_t emergency_stop, uint8_t track_ctrl = 0, uint8_t grag_teach_ctrl = 0,
    uint8_t trajectory_index = 0, uint16_t name_index = 0);

  bool motion_control_2(
    uint8_t ctrl_mode, uint8_t move_mode, uint8_t move_speed_rate, uint8_t is_mit_mode = 0,
    uint8_t residence_time = 0, uint8_t installation_pos = 0);

  // ── Firmware ───────────────────────────────────────────────────────────────

  std::string get_firmware_version();

  // ── SDK joint-limit parameters ─────────────────────────────────────────────

  [[nodiscard]] std::pair<double, double> get_sdk_joint_limit_param(
    const std::string & joint_name);
  [[nodiscard]] std::pair<double, double> get_sdk_gripper_range_param();
  void set_sdk_joint_limit_param(const std::string & joint_name, double min_val, double max_val);
  void set_sdk_gripper_range_param(double min_val, double max_val);

  // ── MIT config ─────────────────────────────────────────────────────────────

  void set_mit_config(const MitConfig & cfg);
  MitConfig get_mit_config() const;

private:
  std::string can_interface_name_;
  bool auto_init_;
  int dh_is_offset_;
  bool enable_sdk_joint_limits_;
  bool enable_sdk_gripper_limits_;

  std::unique_ptr<PiperParams> parameters_;
  std::unique_ptr<PiperProtocolV2> protocol_;  // concrete type for set_mit_config
  std::unique_ptr<CanInterface> can_interface_;

  std::atomic<bool> running_{false};
  std::thread can_monitor_thread_;

  // ── State (protected by individual mutexes for minimal lock contention) ────
  mutable std::mutex arm_status_mutex_;
  mutable std::mutex arm_end_pose_mutex_;
  mutable std::mutex arm_joint_mutex_;
  mutable std::mutex arm_gripper_mutex_;
  mutable std::mutex firmware_mutex_;
  mutable std::mutex motor_info_mutex_;
  mutable std::mutex joint_vel_mutex_;  // joint velocity from 0x481-0x486

  ArmStatus arm_status_;
  ArmEndPose arm_end_pose_;
  ArmJoint arm_joint_;
  ArmGripper arm_gripper_;
  std::vector<uint8_t> firmware_data_;
  std::array<MotorInfo, 6> motor_info_{};
  std::array<double, 6> joint_vel_rad_{};   // joint velocity [rad/s] from vel/acc feedback
  std::array<bool, 6> joint_vel_valid_{};   // true once at least one vel/acc frame arrived

  // ── Private helpers ────────────────────────────────────────────────────────

  void parse_can_frame(CanFrameMsg & frame);
  void can_monitor_loop();

  bool cartesian_ctrl_xy(int x, int y);
  bool cartesian_ctrl_zrx(int z, int rx);
  bool cartesian_ctrl_ryrz(int ry, int rz);
  bool joint_ctrl_12(int j1, int j2);
  bool joint_ctrl_34(int j3, int j4);
  bool joint_ctrl_56(int j5, int j6);

  int check_joint_sdk_limit(int joint_value, const std::string & joint_name);

  /** Build and send a single MIT CAN frame for joint (0-based index). */
  bool send_raw_mit_frame(int joint_idx, const MsgJointMitCtrl & raw);
};

}  // namespace piper
}  // namespace agilex
