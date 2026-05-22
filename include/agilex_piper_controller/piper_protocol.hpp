// ********************************************************************************************************************
// Copyright [2026] Renesas Electronics Corporation and/or its licensors. All Rights Reserved.
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
  PiperProtocolV2();
  ~PiperProtocolV2() override = default;

  ProtocolVersion get_protocol_version() const override;
  bool encode_message(const PiperMessage & msg, CanFrameMsg & frame) override;
  bool decode_message(const CanFrameMsg & frame, PiperMessage & msg) override;

private:
  // Message encoding helper methods
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

  // Message decoding helper methods
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
};

}  // namespace piper
}  // namespace agilex