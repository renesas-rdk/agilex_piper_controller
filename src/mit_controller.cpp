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
#include "agilex_piper_controller/mit_controller.hpp"

#include <chrono>
#include <iostream>
#include <stdexcept>
#include <thread>

namespace agilex
{
namespace piper
{

PiperMitController::PiperMitController(PiperController & robot, double control_freq_hz)
: robot_(robot), control_freq_hz_(control_freq_hz)
{
}

PiperMitController::~PiperMitController() { stop(); }

void PiperMitController::stop() { stop_requested_ = true; }

void PiperMitController::control(ControlCallback callback)
{
  if (!robot_.is_connected()) {
    throw std::runtime_error("PiperMitController::control(): robot is not connected");
  }

  // Enable the arm (all joints)
  if (!robot_.enable_arm()) {
    throw std::runtime_error("PiperMitController::control(): failed to enable arm");
  }

  // Switch to MIT mode (ctrl=0x01 CAN, move=0x04 MOVE_M, mit=0xAD)
  if (!robot_.enable_mit_mode()) {
    throw std::runtime_error("PiperMitController::control(): failed to switch to MIT mode");
  }

  // Allow a short settle time for the mode switch
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  running_ = true;
  stop_requested_ = false;

  const double dt = 1.0 / control_freq_hz_;
  const auto period =
    std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(dt));

  auto next_tick = std::chrono::steady_clock::now();
  std::array<MitJointCommand, 6> cmds{};

  while (!stop_requested_.load(std::memory_order_relaxed)) {
    // Read current state
    RobotState state = robot_.get_robot_state();

    // Call user callback
    bool keep_going = callback(state, dt, cmds);
    if (!keep_going) {
      break;
    }

    // Send all six MIT commands
    robot_.send_mit_cmd_all(cmds);

    // Sleep until next tick
    next_tick += period;
    std::this_thread::sleep_until(next_tick);
  }

  running_ = false;

  // Gracefully switch back to position mode and disable
  robot_.enable_position_mode(50);
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  robot_.disable_arm();
}

}  // namespace piper
}  // namespace agilex
