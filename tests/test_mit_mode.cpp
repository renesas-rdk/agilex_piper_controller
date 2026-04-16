/**
 * MIT-mode (torque control) example for the Agilex Piper robot arm.
 *
 * This example demonstrates the model-based control API:
 *   1. Connect to the robot
 *   2. Switch to MIT mode
 *   3. Run a PD hold loop at 200 Hz using PiperMitController
 *   4. Gracefully shut down
 *
 * Usage:
 *   ./test_mit_mode <can_if> [freq_hz] [duration_s] [kp] [kd]
 *
 * Example:
 *   ./test_mit_mode can0 200 5.0 10.0 0.5
 */
#include "agilex_piper_controller/mit_controller.hpp"
#include "agilex_piper_controller/piper_controller.hpp"

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <iomanip>
#include <iostream>

namespace
{
std::atomic<bool> g_stop{false};

void signal_handler(int) { g_stop = true; }
}  // namespace

int main(int argc, char ** argv)
{
  if (argc < 2) {
    std::cerr << "Usage: " << argv[0] << " <can_if> [freq_hz] [duration_s] [kp] [kd]\n"
              << "Example: " << argv[0] << " can0 200 5.0 10.0 0.5\n";
    return 1;
  }

  const std::string can_if = argv[1];
  const double freq_hz = (argc >= 3) ? std::atof(argv[2]) : 200.0;
  const double duration_s = (argc >= 4) ? std::atof(argv[3]) : 5.0;
  const double kp = (argc >= 5) ? std::atof(argv[4]) : 10.0;
  const double kd = (argc >= 6) ? std::atof(argv[5]) : 0.5;

  std::signal(SIGINT, signal_handler);
  std::signal(SIGTERM, signal_handler);

  // ── 1. Connect ─────────────────────────────────────────────────────────────
  agilex::piper::PiperController robot(can_if);
  if (!robot.is_connected()) {
    std::cerr << "Failed to connect on " << can_if << "\n";
    return 2;
  }

  // ── 2. Capture the starting pose ──────────────────────────────────────────
  // Wait briefly for feedback to arrive
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  auto initial_state = robot.get_robot_state();
  std::array<double, 6> q_hold;
  for (int i = 0; i < 6; ++i) {
    q_hold[i] = initial_state.joints[i].position;
  }

  std::cout << "Initial joint positions [rad]:";
  for (int i = 0; i < 6; ++i) {
    std::cout << " " << std::fixed << std::setprecision(3) << q_hold[i];
  }
  std::cout << "\n";

  std::cout << "Running MIT PD hold at " << freq_hz << " Hz for " << duration_s << " s"
            << " (kp=" << kp << ", kd=" << kd << ")\n"
            << "Press Ctrl+C to stop early.\n\n";

  // ── 3. Run the MIT control loop ───────────────────────────────────────────
  agilex::piper::PiperMitController mit(robot, freq_hz);

  const auto start = std::chrono::steady_clock::now();
  uint64_t cycle_count = 0;

  try {
    mit.control([&](const agilex::piper::RobotState & state, double /*dt*/,
                    std::array<agilex::piper::MitJointCommand, 6> & cmds) -> bool {
      // Check termination
      if (g_stop.load(std::memory_order_relaxed)) {
        return false;
      }
      auto elapsed =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
      if (elapsed >= duration_s) {
        return false;
      }

      // PD hold at the captured position
      for (int i = 0; i < 6; ++i) {
        cmds[i].q_des = q_hold[i];
        cmds[i].dq_des = 0.0;
        cmds[i].kp = kp;
        cmds[i].kd = kd;
        cmds[i].tau_ff = 0.0;  // Insert gravity compensation here
      }

      // Print state every 1 second
      ++cycle_count;
      if (cycle_count % static_cast<uint64_t>(freq_hz) == 0) {
        std::cout << "[t=" << std::fixed << std::setprecision(1) << elapsed << "s] q:";
        for (int i = 0; i < 6; ++i) {
          std::cout << " " << std::setprecision(3) << state.joints[i].position;
        }
        std::cout << "\n";
      }

      return true;
    });
  } catch (const std::exception & e) {
    std::cerr << "MIT control error: " << e.what() << "\n";
    return 3;
  }

  auto total_elapsed =
    std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
  std::cout << "\nDone. " << cycle_count << " cycles in " << std::fixed << std::setprecision(2)
            << total_elapsed << " s ("
            << static_cast<double>(cycle_count) / total_elapsed << " Hz actual)\n";

  return 0;
}
