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

#include <linux/can.h>
#include <net/if.h>
#include <sys/socket.h>

#include <atomic>
#include <functional>
#include <string>
#include <thread>
#include <vector>

namespace agilex
{
namespace piper
{

/**
 * Structure for CAN frame message
 */
struct CanFrameMsg
{
  uint32_t arbitration_id{0};  // CAN ID
  uint8_t data[8]{0};          // Data bytes
  uint8_t dlc{8};              // Data Length Code (0-8)
  bool is_extended_id{false};  // Whether this is an extended ID (29-bit)
};

/**
 * Typedef for CAN frame callback function
 */
using CanFrameCallback = std::function<void(CanFrameMsg &)>;

/**
 * Class for CAN interface
 *
 * This class provides an interface to the CAN bus on the system,
 * allowing for sending and receiving CAN messages.
 */
class CanInterface
{
public:
  /**
   * Constructor
   *
   * Parameters:
   *   interface_name - Name of the CAN interface (e.g., "can0")
   *   bitrate - Bitrate in bits per second (default: 1000000 for 1 Mbps)
   *   callback - Callback function for received frames
   */
  explicit CanInterface(
    const std::string & interface_name, int bitrate = 1000000, CanFrameCallback callback = nullptr);

  /**
   * Destructor - cleans up resources and stops the receive thread
   */
  ~CanInterface();

  /**
   * Initialize the CAN interface
   *
   * Sets up the CAN socket, configures the interface and prepares for communication
   *
   * Returns true if successful, false otherwise
   */
  bool initialize();

  /**
   * Start the receive thread
   *
   * Launches a thread that continually reads CAN frames from the bus
   *
   * Returns true if successful, false otherwise
   */
  bool start();

  /**
   * Stop the receive thread
   *
   * Stops the receiver thread and closes the socket
   */
  void stop();

  /**
   * Check if the CAN interface is running
   *
   * Returns true if running, false otherwise
   */
  bool is_running() const;

  /**
   * Send a CAN message
   *
   * Parameters:
   *   id - CAN ID
   *   data - Data bytes
   *   dlc - Data length code (0-8)
   *
   * Returns true if successful, false otherwise
   */
  bool send_message(uint32_t id, const uint8_t * data, uint8_t dlc);

  /**
   * Send a CAN message
   *
   * Parameters:
   *   msg - CAN frame message
   *
   * Returns true if successful, false otherwise
   */
  bool send_message(const CanFrameMsg & msg);

private:
  std::string interface_name_;  // Interface name
  int socket_fd_{-1};           // Socket file descriptor
  int bitrate_;                 // CAN bitrate
  struct sockaddr_can addr_;    // Socket address
  struct ifreq ifr_;            // Interface request structure

  std::atomic<bool> running_{false};  // Whether the receive thread is running
  CanFrameCallback callback_;         // Callback for received frames
  std::thread read_thread_;           // Thread for reading CAN frames
  int lock_fd_;                       // File descriptor for interface configuration lock

  /**
   * Thread function for receiving frames
   */
  void read_loop();

  /**
   * Set the CAN interface state (up or down)
   *
   * Parameters:
   *   up - True to set interface up, false to set interface down
   *
   * Returns true if successful, false otherwise
   */
  bool set_interface_state(bool up);

  /**
   * Set the CAN interface baudrate
   *
   * Parameters:
   *   baudrate - Baudrate in bits per second
   *
   * Returns true if successful, false otherwise
   */
  bool set_interface_baudrate(int baudrate);

  /**
   * Release the lock on the CAN interface
   */
  void release_interface_lock();
};

}  // namespace piper
}  // namespace agilex