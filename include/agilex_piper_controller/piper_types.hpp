#pragma once

// Standard library includes
#include <cstdint>
#include <map>
#include <string>
#include <utility>  // For std::pair
#include <vector>

namespace agilex
{
namespace piper
{

/**
 * Structure for arm status
 */
struct ArmStatus
{
  uint8_t ctrl_mode{ 0 };
  uint8_t arm_status{ 0 };
  uint8_t mode_feed{ 0 };
  uint8_t teach_status{ 0 };
  uint8_t motion_status{ 0 };
  uint8_t trajectory_num{ 0 };
  uint8_t err_code_comm{ 0 };
  uint8_t err_code_angle{ 0 };
};

/**
 * Structure for arm end pose (position and orientation)
 */
struct ArmEndPose
{
  int x{ 0 };   // X position (0.001mm)
  int y{ 0 };   // Y position (0.001mm)
  int z{ 0 };   // Z position (0.001mm)
  int rx{ 0 };  // RX rotation (0.001 degrees)
  int ry{ 0 };  // RY rotation (0.001 degrees)
  int rz{ 0 };  // RZ rotation (0.001 degrees)
};

/**
 * Structure for arm joint angles
 */
struct ArmJoint
{
  int j1{ 0 };  // Joint 1 angle (0.001 degrees)
  int j2{ 0 };  // Joint 2 angle (0.001 degrees)
  int j3{ 0 };  // Joint 3 angle (0.001 degrees)
  int j4{ 0 };  // Joint 4 angle (0.001 degrees)
  int j5{ 0 };  // Joint 5 angle (0.001 degrees)
  int j6{ 0 };  // Joint 6 angle (0.001 degrees)

  /**
   * Convert joint angles to vector
   *
   * Returns Vector of joint angles
   */
  std::vector<int> to_vector() const
  {
    return { j1, j2, j3, j4, j5, j6 };
  }

  /**
   * Set joint angles from vector
   *
   * Parameters:
   *   vec - Vector of joint angles
   */
  void from_vector(const std::vector<int>& vec)
  {
    if (vec.size() >= 6)
    {
      j1 = vec[0];
      j2 = vec[1];
      j3 = vec[2];
      j4 = vec[3];
      j5 = vec[4];
      j6 = vec[5];
    }
  }
};

/**
 * Structure for arm gripper state
 */
struct ArmGripper
{
  int32_t grippers_angle{ 0 };    // Gripper position
  uint16_t grippers_effort{ 0 };  // Gripper speed
  uint8_t status_code{ 0 };       // Control mode
  uint8_t set_zero{ 0 };          // Zero set flag
};

/**
 * Structure for motor information
 */
struct MotorInfo
{
  int16_t motor_speed;        // Motor speed (0.001rad/s)
  uint16_t current;           // Motor current (0.001A)
  int32_t position;           // Motor position (rad)
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
 * Enum for control modes
 */
enum ControlMode
{
  CTRL_MODE_NOT_SPECIFIED = 0,  // Not specified
  CTRL_MODE_POSITION = 1,       // Position control
  CTRL_MODE_VELOCITY = 2,       // Velocity control
  CTRL_MODE_TORQUE = 3          // Torque control
};

/**
 * Enum for movement modes
 */
enum MoveMode
{
  MOVE_MODE_NOT_SPECIFIED = 0,  // Not specified
  MOVE_MODE_JOINT = 1,          // Joint space movement
  MOVE_MODE_CARTESIAN = 2       // Cartesian space movement
};

/**
 * Enum for motion status
 */
enum MotionStatus
{
  MOTION_STOPPED = 0,  // Stopped
  MOTION_MOVING = 1,   // Moving
  MOTION_PAUSED = 2,   // Paused
  MOTION_ERROR = 3     // Error
};

/**
 * Enum for error status
 */
enum ErrorStatus
{
  ERROR_NONE = 0,       // No error
  ERROR_CAN = 1,        // CAN error
  ERROR_HARDWARE = 2,   // Hardware error
  ERROR_LIMIT = 3,      // Limit error
  ERROR_COLLISION = 4,  // Collision error
  ERROR_SOFTWARE = 5    // Software error
};

/**
 * Enum for warning status
 */
enum WarningStatus
{
  WARNING_NONE = 0,         // No warning
  WARNING_LOW_BATTERY = 1,  // Low battery
  WARNING_HIGH_TEMP = 2,    // High temperature
  WARNING_MOTOR_LOAD = 3    // Motor load warning
};

/**
 * Structure for joint limits
 */
struct JointLimits
{
  double min{ 0.0 };  // Minimum angle (radians)
  double max{ 0.0 };  // Maximum angle (radians)
};

/**
 * Class for Piper parameters
 */
class PiperParams
{
public:
  /**
   * Constructor
   */
  PiperParams();

  /**
   * Destructor
   */
  ~PiperParams() = default;

  /**
   * Reset parameters to default values
   */
  void reset_default_params();

  /**
   * Get joint limits for a specific joint
   *
   * Parameters:
   *   joint_name - Joint name (j1, j2, etc.)
   *
   * Returns joint limits
   */
  [[nodiscard]] JointLimits get_joint_limits(const std::string& joint_name) const;

  /**
   * Set joint limits
   *
   * Parameters:
   *   joint_name - Joint name (j1, j2, etc.)
   *   min - Minimum angle (radians)
   *   max - Maximum angle (radians)
   */
  void set_joint_limits(const std::string& joint_name, double min, double max);

  /**
   * Get gripper range
   *
   * Returns gripper range limits
   */
  [[nodiscard]] JointLimits get_gripper_range() const;

  /**
   * Set gripper range
   *
   * Parameters:
   *   min - Minimum position
   *   max - Maximum position
   */
  void set_gripper_range(double min, double max);

  /**
   * Enable SDK joint limits
   *
   * Parameters:
   *   enable - Whether to enable SDK joint limits
   */
  void enable_sdk_joint_limits(bool enable);

  /**
   * Enable SDK gripper limits
   *
   * Parameters:
   *   enable - Whether to enable SDK gripper limits
   */
  void enable_sdk_gripper_limits(bool enable);

  /**
   * Check if SDK joint limits are enabled
   *
   * Returns true if enabled, false otherwise
   */
  [[nodiscard]] bool is_sdk_joint_limits_enabled() const;

  /**
   * Check if SDK gripper limits are enabled
   *
   * Returns true if enabled, false otherwise
   */
  [[nodiscard]] bool is_sdk_gripper_limits_enabled() const;

private:
  // Joint limits (radians) stored in a map
  std::map<std::string, std::pair<double, double>> joint_limits_;

  // Gripper range as a pair (min, max)
  std::pair<double, double> gripper_range_;

  // Whether SDK limits are enabled
  bool enable_sdk_joint_limits_{ true };
  bool enable_sdk_gripper_limits_{ true };
};

}  // namespace piper
}  // namespace agilex