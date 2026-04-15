#include "agilex_piper_controller/piper_controller.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <limits>
#include <string>
#include <thread>
#include <vector>

namespace
{
std::atomic<bool> g_stop{false};

void signal_handler(int)
{
  g_stop = true;
}

struct StepResult
{
  double target_hz{0.0};
  double actual_hz{0.0};
  uint64_t ok{0};
  uint64_t fail{0};
  double fail_ratio_pct{0.0};
  bool stable{false};
};

StepResult run_step(
  agilex::piper::PiperController & controller, double target_hz, int duration_sec, int amplitude_mdeg,
  double max_fail_ratio_pct)
{
  StepResult result;
  result.target_hz = target_hz;

  const auto period = std::chrono::duration<double>(1.0 / target_hz);
  auto next_tick = std::chrono::steady_clock::now();
  const auto start = next_tick;
  const auto end_time = start + std::chrono::seconds(duration_sec);
  bool positive = true;

  while (!g_stop.load(std::memory_order_relaxed) && std::chrono::steady_clock::now() < end_time) {
    const int j1 = positive ? amplitude_mdeg : -amplitude_mdeg;
    const int j2 = -j1;
    positive = !positive;

    if (controller.set_joint_angles(j1, j2, 0, 0, 0, 0)) {
      ++result.ok;
    } else {
      ++result.fail;
    }

    next_tick += std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);
    std::this_thread::sleep_until(next_tick);
  }

  const auto finish = std::chrono::steady_clock::now();
  const double elapsed_s =
    std::chrono::duration_cast<std::chrono::duration<double>>(finish - start).count();
  const uint64_t total = result.ok + result.fail;
  result.actual_hz = (elapsed_s > 0.0) ? (static_cast<double>(total) / elapsed_s) : 0.0;
  result.fail_ratio_pct = (total > 0) ? (100.0 * static_cast<double>(result.fail) / static_cast<double>(total))
                                      : 100.0;
  result.stable = (total > 0) && (result.fail_ratio_pct <= max_fail_ratio_pct) &&
                  (result.actual_hz >= target_hz * 0.98);
  return result;
}

}  // namespace

int main(int argc, char ** argv)
{
  if (argc < 2) {
    std::cerr << "Usage: " << argv[0]
              << " <can_if> [step_duration_sec] [joint_amplitude_mdeg] [start_hz] [end_hz] "
                 "[step_hz] [max_fail_ratio_pct]\n";
    std::cerr << "Example: " << argv[0] << " can0 3 1000 20 300 20 1.0\n";
    return 1;
  }

  const std::string can_if = argv[1];
  const int step_duration_sec = (argc >= 3) ? std::max(1, std::atoi(argv[2])) : 3;
  const int joint_amplitude_mdeg = (argc >= 4) ? std::max(1, std::atoi(argv[3])) : 1000;
  const double start_hz = (argc >= 5) ? std::max(1.0, std::atof(argv[4])) : 20.0;
  const double end_hz = (argc >= 6) ? std::max(start_hz, std::atof(argv[5])) : 300.0;
  const double step_hz = (argc >= 7) ? std::max(1.0, std::atof(argv[6])) : 20.0;
  const double max_fail_ratio_pct = (argc >= 8) ? std::max(0.0, std::atof(argv[7])) : 1.0;
  const int speed_rate = 100;

  std::signal(SIGINT, signal_handler);
  std::signal(SIGTERM, signal_handler);

  agilex::piper::PiperController controller(can_if);
  if (!controller.is_connected()) {
    std::cerr << "Failed to connect PiperController on " << can_if
              << ". Check interface/permissions/robot power.\n";
    return 2;
  }

  if (!controller.enable_arm()) {
    std::cerr << "Failed to enable arm.\n";
    return 3;
  }

  if (!controller.set_mode(0x01, 0x01, static_cast<uint8_t>(speed_rate))) {
    std::cerr << "Failed to set controller mode to CAN joint control.\n";
    return 4;
  }

  std::cout << "Running stable joint-command frequency sweep on " << can_if << "\n";
  std::cout << "Step duration: " << step_duration_sec << " s, amplitude: " << joint_amplitude_mdeg
            << " mdeg, fail threshold: " << max_fail_ratio_pct << "%\n";
  std::cout << "Sweep range: " << start_hz << " -> " << end_hz << " Hz, step " << step_hz << " Hz\n";
  std::cout << "Press Ctrl+C to stop early.\n\n";

  std::vector<StepResult> results;
  double max_stable_hz = 0.0;

  for (double hz = start_hz; hz <= end_hz + std::numeric_limits<double>::epsilon(); hz += step_hz) {
    if (g_stop.load(std::memory_order_relaxed)) {
      break;
    }

    auto step_result =
      run_step(controller, hz, step_duration_sec, joint_amplitude_mdeg, max_fail_ratio_pct);
    results.push_back(step_result);

    if (step_result.stable) {
      max_stable_hz = step_result.target_hz;
    } else {
      break;
    }

    // Small gap between two frequency levels to reduce carry-over queue pressure.
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }

  controller.disable_arm();
  controller.disconnect();

  std::cout << std::fixed << std::setprecision(2);
  std::cout << "=== Stable Frequency Sweep Result ===\n";
  std::cout << "TargetHz  ActualHz  OK  FAIL  Fail%  Stable\n";
  for (const auto & r : results) {
    std::cout << std::setw(8) << r.target_hz << "  " << std::setw(8) << r.actual_hz << "  " << std::setw(6)
              << r.ok << "  " << std::setw(6) << r.fail << "  " << std::setw(6) << r.fail_ratio_pct
              << "  " << (r.stable ? "YES" : "NO") << "\n";
  }

  if (max_stable_hz > 0.0) {
    std::cout << "\nMax stable joint command frequency: " << max_stable_hz << " Hz\n";
  } else {
    std::cout << "\nNo stable frequency found in the given range.\n";
  }

  return 0;
}
