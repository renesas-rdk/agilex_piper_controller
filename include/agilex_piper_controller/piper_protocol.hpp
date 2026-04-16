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

#include <cstdint>
#include <memory>
#include <vector>

#include "agilex_piper_controller/can_interface.hpp"
#include "agilex_piper_controller/piper_messages.hpp"
#include "agilex_piper_controller/piper_types.hpp"

namespace agilex
{
namespace piper
{

// ─────────────────────────────────────────────────────────────────────────────
// Fixed-point helpers for MIT CAN encoding
// ─────────────────────────────────────────────────────────────────────────────

/**
 * Map a float in [x_min, x_max] to an N-bit unsigned integer [0, 2^bits - 1].
 */
inline uint16_t float_to_uint(double x, double x_min, double x_max, int bits)
{
  double span = x_max - x_min;
  uint16_t max_val = static_cast<uint16_t>((1u << bits) - 1u);
  int32_t raw = static_cast<int32_t>((x - x_min) / span * max_val + 0.5);
  if (raw < 0) raw = 0;
  if (raw > static_cast<int32_t>(max_val)) raw = static_cast<int32_t>(max_val);
  return static_cast<uint16_t>(raw);
}

/**
 * Inverse of float_to_uint: decode an N-bit integer back to a float in [x_min, x_max].
 */
inline double uint_to_float(uint16_t x_int, double x_min, double x_max, int bits)
{
  uint16_t max_val = static_cast<uint16_t>((1u << bits) - 1u);
  return static_cast<double>(x_int) / max_val * (x_max - x_min) + x_min;
}

// ─────────────────────────────────────────────────────────────────────────────
// Protocol classes
// ─────────────────────────────────────────────────────────────────────────────

enum class ProtocolVersion
{
  V1,
  V2
};

class PiperProtocolBase
{
public:
  virtual ~PiperProtocolBase() = default;

  virtual ProtocolVersion get_protocol_version() const = 0;
  virtual bool encode_message(const PiperMessage & msg, CanFrameMsg & frame) = 0;
  virtual bool decode_message(const CanFrameMsg & frame, PiperMessage & msg) = 0;

  // Static helper functions for byte conversion
  static int32_t bytes_to_int32(const uint8_t * data);
  static int16_t bytes_to_int16(const uint8_t * data);
  static uint32_t bytes_to_uint32(const uint8_t * data);
  static uint16_t bytes_to_uint16(const uint8_t * data);
  static void int32_to_bytes(int32_t value, uint8_t * data);
  static void int16_to_bytes(int16_t value, uint8_t * data);
  static void uint32_to_bytes(uint32_t value, uint8_t * data);
  static void uint16_to_bytes(uint16_t value, uint8_t * data);
};

class PiperProtocolV2 : public PiperProtocolBase
{
public:
  explicit PiperProtocolV2(const MitConfig & mit_cfg = MitConfig{});
  ~PiperProtocolV2() override = default;

  ProtocolVersion get_protocol_version() const override;
  bool encode_message(const PiperMessage & msg, CanFrameMsg & frame) override;
  bool decode_message(const CanFrameMsg & frame, PiperMessage & msg) override;

  /** Update MIT encoding limits at runtime (e.g. per-robot calibration). */
  void set_mit_config(const MitConfig & cfg) { mit_cfg_ = cfg; }
  const MitConfig & get_mit_config() const { return mit_cfg_; }

private:
  MitConfig mit_cfg_;  ///< MIT fixed-point encoding limits

  // Encoding helpers
  bool encode_enable_disable_arm(const PiperMessage & msg, CanFrameMsg & frame);
  bool encode_motion_ctrl_1(const PiperMessage & msg, CanFrameMsg & frame);
  bool encode_motion_ctrl_2(const PiperMessage & msg, CanFrameMsg & frame);
  bool encode_cartesian_ctrl_1(const PiperMessage & msg, CanFrameMsg & frame);
  bool encode_cartesian_ctrl_2(const PiperMessage & msg, CanFrameMsg & frame);
  bool encode_cartesian_ctrl_3(const PiperMessage & msg, CanFrameMsg & frame);
  bool encode_joint_ctrl_12(const PiperMessage & msg, CanFrameMsg & frame);
  bool encode_joint_ctrl_34(const PiperMessage & msg, CanFrameMsg & frame);
  bool encode_joint_ctrl_56(const PiperMessage & msg, CanFrameMsg & frame);
  bool encode_gripper_ctrl(const PiperMessage & msg, CanFrameMsg & frame);
  bool encode_joint_config(const PiperMessage & msg, CanFrameMsg & frame);
  bool encode_crash_protection_config(const PiperMessage & msg, CanFrameMsg & frame);
  bool encode_master_slave_config(const PiperMessage & msg, CanFrameMsg & frame);
  bool encode_circular_pattern_coord_update(const PiperMessage & msg, CanFrameMsg & frame);
  /**
   * Encode a single MIT-mode joint command.
   *
   * Bit layout (64 bits, big-endian):
   *   [63:48] pos  16-bit  ±pos_max rad
   *   [47:36] vel  12-bit  ±vel_max rad/s
   *   [35:24] kp   12-bit   0…kp_max Nm/rad
   *   [23:12] kd   12-bit   0…kd_max Nms/rad
   *   [11:0]  tau  12-bit  ±tau_max Nm
   */
  bool encode_joint_mit_ctrl(const PiperMessage & msg, CanFrameMsg & frame);

  // Decoding helpers
  bool decode_arm_status(const CanFrameMsg & frame, PiperMessage & msg);
  bool decode_end_pose_xy(const CanFrameMsg & frame, PiperMessage & msg);
  bool decode_end_pose_zrx(const CanFrameMsg & frame, PiperMessage & msg);
  bool decode_end_pose_ryrz(const CanFrameMsg & frame, PiperMessage & msg);
  bool decode_joint_12(const CanFrameMsg & frame, PiperMessage & msg);
  bool decode_joint_34(const CanFrameMsg & frame, PiperMessage & msg);
  bool decode_joint_56(const CanFrameMsg & frame, PiperMessage & msg);
  bool decode_gripper(const CanFrameMsg & frame, PiperMessage & msg);
  bool decode_motor_info_high_spd(const CanFrameMsg & frame, PiperMessage & msg);
  bool decode_motor_info_low_spd(const CanFrameMsg & frame, PiperMessage & msg);
  bool decode_firmware_version(const CanFrameMsg & frame, PiperMessage & msg);
  bool decode_joint_vel_acc(const CanFrameMsg & frame, PiperMessage & msg);
};

}  // namespace piper
}  // namespace agilex
