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
#include <functional>

#include "agilex_piper_controller/piper_controller.hpp"
#include "agilex_piper_controller/piper_types.hpp"

namespace agilex
{
namespace piper
{

/**
 * High-level MIT (torque) mode controller for the Piper robot arm.
 *
 * Inspired by libfranka's Robot::control() and arx5-sdk's JointController.
 *
 * Usage:
 *
 *   PiperController robot("can0");
 *   PiperMitController mit(robot, 200.0);
 *
 *   // PD hold at current position
 *   mit.control([](const RobotState& state, double dt,
 *                   std::array<MitJointCommand,6>& cmds) -> bool {
 *       for (int i = 0; i < 6; ++i) {
 *           cmds[i].q_des  = state.joints[i].position;  // hold
 *           cmds[i].dq_des = 0.0;
 *           cmds[i].kp     = 10.0;
 *           cmds[i].kd     = 0.5;
 *           cmds[i].tau_ff = 0.0;  // add gravity comp here
 *       }
 *       return true;   // return false to stop the loop
 *   });
 *
 * The controller handles:
 *   - Switching the robot to MIT mode (MOVE M + 0xAD)
 *   - Running a fixed-frequency loop that calls the user callback
 *   - Sending the six MIT CAN frames each cycle
 *   - Switching back to position mode when the loop ends
 *
 * The user callback is responsible for computing the desired joint commands
 * (position, velocity, gains, feedforward torque). This is where model-based
 * controllers (gravity compensation, impedance control, etc.) are implemented.
 */
class PiperMitController
{
public:
  /**
   * Callback signature for the control loop.
   *
   * Parameters:
   *   state  - Current robot state snapshot (SI units)
   *   dt     - Nominal time step [s] (= 1.0 / control_freq_hz)
   *   cmds   - [out] Six joint commands to send this cycle
   *
   * Returns:
   *   true  → continue the loop
   *   false → exit the loop gracefully
   */
  using ControlCallback = std::function<bool(
    const RobotState & state, double dt, std::array<MitJointCommand, 6> & cmds)>;

  /**
   * Constructor.
   *
   * Parameters:
   *   robot           - A connected PiperController (not owned)
   *   control_freq_hz - Desired loop frequency [Hz] (default 200)
   */
  PiperMitController(PiperController & robot, double control_freq_hz = 200.0);

  ~PiperMitController();

  /**
   * Run the MIT control loop (blocking).
   *
   * Enables the arm, switches to MIT mode, calls the callback at the
   * configured frequency, and disables MIT mode when done.
   *
   * Throws std::runtime_error if the robot is not connected or mode
   * switch fails.
   */
  void control(ControlCallback callback);

  /** Request the control loop to stop (thread-safe). */
  void stop();

  /** Returns true while the control loop is running. */
  bool is_running() const { return running_.load(); }

  /** Change the control frequency (only takes effect before next control() call). */
  void set_control_freq(double hz) { control_freq_hz_ = hz; }

  /** Get the control frequency. */
  double get_control_freq() const { return control_freq_hz_; }

private:
  PiperController & robot_;
  double control_freq_hz_;
  std::atomic<bool> running_{false};
  std::atomic<bool> stop_requested_{false};
};

}  // namespace piper
}  // namespace agilex
