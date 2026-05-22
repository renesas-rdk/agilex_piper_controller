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
#include <cmath>

#include "agilex_piper_controller/piper_types.hpp"

namespace agilex
{
namespace piper
{

PiperParams::PiperParams()
: joint_limits_(),
  gripper_range_(),
  enable_sdk_joint_limits_(true),
  enable_sdk_gripper_limits_(true)
{
  reset_default_params();
}

void PiperParams::reset_default_params()
{
  // Set default joint limits (in radians)
  // Values based on the Piper robotic arm specifications

  // Joint 1 limits (base joint)
  joint_limits_["joint1"] = std::make_pair(-M_PI, M_PI);  // -180° to +180°

  // Joint 2 limits (shoulder joint)
  joint_limits_["joint2"] = std::make_pair(-M_PI / 2, M_PI / 2);  // -90° to +90°

  // Joint 3 limits (elbow joint)
  joint_limits_["joint3"] = std::make_pair(-M_PI * 0.8, M_PI * 0.8);  // -145° to +145°

  // Joint 4 limits (wrist 1 joint)
  joint_limits_["joint4"] = std::make_pair(-M_PI, M_PI);  // -180° to +180°

  // Joint 5 limits (wrist 2 joint)
  joint_limits_["joint5"] = std::make_pair(-M_PI / 2, M_PI / 2);  // -90° to +90°

  // Joint 6 limits (wrist 3 joint)
  joint_limits_["joint6"] = std::make_pair(-M_PI, M_PI);  // -180° to +180°

  // Set default gripper range (in meters)
  gripper_range_ = std::make_pair(0.0, 0.08);  // 0 to 8cm opening
}

// Modern method implementations

JointLimits PiperParams::get_joint_limits(const std::string & joint_name) const
{
  JointLimits limits;
  if (joint_limits_.find(joint_name) != joint_limits_.end()) {
    auto pair = joint_limits_.at(joint_name);
    limits.min = pair.first;
    limits.max = pair.second;
  } else {
    // Default limits if joint not found
    limits.min = -M_PI;
    limits.max = M_PI;
  }
  return limits;
}

void PiperParams::set_joint_limits(const std::string & joint_name, double min, double max)
{
  if (min <= max) {
    joint_limits_[joint_name] = std::make_pair(min, max);
  }
}

JointLimits PiperParams::get_gripper_range() const
{
  JointLimits limits;
  limits.min = gripper_range_.first;
  limits.max = gripper_range_.second;
  return limits;
}

void PiperParams::set_gripper_range(double min, double max)
{
  if (min <= max) {
    gripper_range_ = std::make_pair(min, max);
  }
}

void PiperParams::enable_sdk_joint_limits(bool enable) { enable_sdk_joint_limits_ = enable; }

void PiperParams::enable_sdk_gripper_limits(bool enable) { enable_sdk_gripper_limits_ = enable; }

bool PiperParams::is_sdk_joint_limits_enabled() const { return enable_sdk_joint_limits_; }

bool PiperParams::is_sdk_gripper_limits_enabled() const { return enable_sdk_gripper_limits_; }

}  // namespace piper
}  // namespace agilex