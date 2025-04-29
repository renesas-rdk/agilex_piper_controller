#pragma once

#include <cstdint>
#include <vector>
#include <memory>

#include "agilex_piper_controller/piper_types.hpp"
#include "agilex_piper_controller/piper_messages.hpp"
#include "agilex_piper_controller/can_interface.hpp"

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
  virtual bool encode_message(const PiperMessage& msg, CanFrameMsg& frame) = 0;
  virtual bool decode_message(const CanFrameMsg& frame, PiperMessage& msg) = 0;

  // Static helper functions for byte conversion
  static int32_t bytes_to_int32(const uint8_t* data);
  static int16_t bytes_to_int16(const uint8_t* data);
  static uint32_t bytes_to_uint32(const uint8_t* data);
  static uint16_t bytes_to_uint16(const uint8_t* data);
  static void int32_to_bytes(int32_t value, uint8_t* data);
  static void int16_to_bytes(int16_t value, uint8_t* data);
  static void uint32_to_bytes(uint32_t value, uint8_t* data);
  static void uint16_to_bytes(uint16_t value, uint8_t* data);
};

class PiperProtocolV2 : public PiperProtocolBase
{
public:
  PiperProtocolV2();
  ~PiperProtocolV2() override = default;

  ProtocolVersion get_protocol_version() const override;
  bool encode_message(const PiperMessage& msg, CanFrameMsg& frame) override;
  bool decode_message(const CanFrameMsg& frame, PiperMessage& msg) override;

private:
  // Message encoding helper methods
  bool encode_enable_disable_arm(const PiperMessage& msg, CanFrameMsg& frame);
  bool encode_motion_ctrl_1(const PiperMessage& msg, CanFrameMsg& frame);
  bool encode_motion_ctrl_2(const PiperMessage& msg, CanFrameMsg& frame);
  bool encode_cartesian_ctrl_1(const PiperMessage& msg, CanFrameMsg& frame);
  bool encode_cartesian_ctrl_2(const PiperMessage& msg, CanFrameMsg& frame);
  bool encode_cartesian_ctrl_3(const PiperMessage& msg, CanFrameMsg& frame);
  bool encode_joint_ctrl_12(const PiperMessage& msg, CanFrameMsg& frame);
  bool encode_joint_ctrl_34(const PiperMessage& msg, CanFrameMsg& frame);
  bool encode_joint_ctrl_56(const PiperMessage& msg, CanFrameMsg& frame);
  bool encode_gripper_ctrl(const PiperMessage& msg, CanFrameMsg& frame);
  bool encode_joint_config(const PiperMessage& msg, CanFrameMsg& frame);
  bool encode_crash_protection_config(const PiperMessage& msg, CanFrameMsg& frame);
  bool encode_master_slave_config(const PiperMessage& msg, CanFrameMsg& frame);
  bool encode_circular_pattern_coord_update(const PiperMessage& msg, CanFrameMsg& frame);

  // Message decoding helper methods
  bool decode_arm_status(const CanFrameMsg& frame, PiperMessage& msg);
  bool decode_end_pose_xy(const CanFrameMsg& frame, PiperMessage& msg);
  bool decode_end_pose_zrx(const CanFrameMsg& frame, PiperMessage& msg);
  bool decode_end_pose_ryrz(const CanFrameMsg& frame, PiperMessage& msg);
  bool decode_joint_12(const CanFrameMsg& frame, PiperMessage& msg);
  bool decode_joint_34(const CanFrameMsg& frame, PiperMessage& msg);
  bool decode_joint_56(const CanFrameMsg& frame, PiperMessage& msg);
  bool decode_gripper(const CanFrameMsg& frame, PiperMessage& msg);
  bool decode_motor_info_high_spd(const CanFrameMsg& frame, PiperMessage& msg);
  bool decode_motor_info_low_spd(const CanFrameMsg& frame, PiperMessage& msg);
  bool decode_firmware_version(const CanFrameMsg& frame, PiperMessage& msg);
};

}  // namespace piper
}  // namespace agilex