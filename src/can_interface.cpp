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
#include "agilex_piper_controller/can_interface.hpp"

#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <linux/sockios.h>
#include <net/if.h>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstring>
#include <fstream>
#include <iostream>
#include <string>

namespace agilex
{
namespace piper
{

CanInterface::CanInterface(
  const std::string & interface_name, int bitrate, CanFrameCallback callback)
: interface_name_(interface_name), bitrate_(bitrate), callback_(callback), lock_fd_(-1)
{
  memset(&addr_, 0, sizeof(addr_));
  memset(&ifr_, 0, sizeof(ifr_));
}

CanInterface::~CanInterface()
{
  stop();
  release_interface_lock();
}

bool CanInterface::initialize()
{
  // Open the socket
  if ((socket_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
    std::cerr << "Error opening CAN socket: " << strerror(errno) << std::endl;
    return false;
  }

  // Get the interface index
  strncpy(ifr_.ifr_name, interface_name_.c_str(), IFNAMSIZ - 1);
  if (ioctl(socket_fd_, SIOCGIFINDEX, &ifr_) < 0) {
    std::cerr << "Error getting interface index for " << interface_name_ << ": " << strerror(errno)
              << std::endl;
    close(socket_fd_);
    socket_fd_ = -1;
    return false;
  }

  // Try to acquire exclusive lock for interface configuration
  std::string lock_file = "/var/lock/can_" + interface_name_ + ".lock";
  std::string ready_file = "/var/lock/can_" + interface_name_ + ".ready";

  lock_fd_ = open(lock_file.c_str(), O_CREAT | O_RDWR, 0644);
  if (lock_fd_ < 0) {
    std::cerr << "Error creating lock file: " << strerror(errno) << std::endl;
    close(socket_fd_);
    socket_fd_ = -1;
    return false;
  }

  // Try to acquire exclusive lock (non-blocking first to check)
  if (flock(lock_fd_, LOCK_EX | LOCK_NB) == 0) {
    // We got the lock immediately, so we're the first process
    std::cout << "First process to configure CAN interface " << interface_name_ << std::endl;

    // Remove any stale ready file
    unlink(ready_file.c_str());

    // Set down the CAN interface to configure baudrate
    if (!set_interface_state(false)) {
      std::cerr << "Error setting interface " << interface_name_ << " down" << std::endl;
      release_interface_lock();
      close(socket_fd_);
      socket_fd_ = -1;
      return false;
    }

    // Set the baudrate using system command
    if (!set_interface_baudrate(bitrate_)) {
      std::cerr << "Error setting interface " << interface_name_ << " baudrate to " << bitrate_
                << std::endl;
      release_interface_lock();
      close(socket_fd_);
      socket_fd_ = -1;
      return false;
    }

    // Set up the CAN interface after configuring baudrate
    if (!set_interface_state(true)) {
      std::cerr << "Error setting interface " << interface_name_ << " up" << std::endl;
      release_interface_lock();
      close(socket_fd_);
      socket_fd_ = -1;
      return false;
    }

    // Write our PID to the lock file to indicate who configured it
    std::string pid_str = std::to_string(getpid()) + "\n";
    lseek(lock_fd_, 0, SEEK_SET);
    if (ftruncate(lock_fd_, 0) == -1) {
      std::cerr << "Warning: Failed to truncate lock file" << std::endl;
    }
    if (write(lock_fd_, pid_str.c_str(), pid_str.length()) == -1) {
      std::cerr << "Warning: Failed to write PID to lock file" << std::endl;
    }

    // Create ready file to signal initialization is complete
    int ready_fd = open(ready_file.c_str(), O_CREAT | O_WRONLY, 0644);
    if (ready_fd >= 0) {
      if (write(ready_fd, "1", 1) == -1) {
        std::cerr << "Warning: Failed to write to ready file" << std::endl;
      }
      close(ready_fd);
    }

    std::cout << "CAN interface " << interface_name_ << " configuration complete" << std::endl;

  } else if (errno == EWOULDBLOCK) {
    // Another process has the lock, wait for it to complete
    std::cout << "Waiting for another process to configure CAN interface " << interface_name_
              << std::endl;

    // Release our lock fd and wait
    close(lock_fd_);
    lock_fd_ = -1;

    // Wait for the ready file to appear (with timeout)
    int wait_count = 0;
    const int max_wait = 100;  // 10 seconds (100 * 100ms)

    while (wait_count < max_wait) {
      if (access(ready_file.c_str(), F_OK) == 0) {
        std::cout << "CAN interface " << interface_name_
                  << " is ready (configured by another process)" << std::endl;
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      wait_count++;
    }

    if (wait_count >= max_wait) {
      std::cerr << "Timeout waiting for CAN interface " << interface_name_ << " to be configured"
                << std::endl;
      close(socket_fd_);
      socket_fd_ = -1;
      return false;
    }

  } else {
    std::cerr << "Error checking lock: " << strerror(errno) << std::endl;
    close(lock_fd_);
    lock_fd_ = -1;
    close(socket_fd_);
    socket_fd_ = -1;
    return false;
  }

  // Bind the socket to the CAN interface
  addr_.can_family = AF_CAN;
  addr_.can_ifindex = ifr_.ifr_ifindex;
  if (bind(socket_fd_, (struct sockaddr *)&addr_, sizeof(addr_)) < 0) {
    std::cerr << "Error binding socket to interface " << interface_name_ << ": " << strerror(errno)
              << std::endl;
    release_interface_lock();
    close(socket_fd_);
    socket_fd_ = -1;
    return false;
  }

  // Set non-blocking mode
  int flags = fcntl(socket_fd_, F_GETFL, 0);
  if (flags < 0) {
    std::cerr << "Error getting socket flags: " << strerror(errno) << std::endl;
    close(socket_fd_);
    socket_fd_ = -1;
    return false;
  }

  if (fcntl(socket_fd_, F_SETFL, flags | O_NONBLOCK) < 0) {
    std::cerr << "Error setting socket to non-blocking mode: " << strerror(errno) << std::endl;
    close(socket_fd_);
    socket_fd_ = -1;
    return false;
  }

  return true;
}

bool CanInterface::set_interface_state(bool up)
{
  std::string cmd = "sudo ip link set " + interface_name_ + (up ? " up" : " down");
  int ret = system(cmd.c_str());
  return (ret == 0);
}

bool CanInterface::set_interface_baudrate(int baudrate)
{
  std::string cmd =
    "sudo ip link set " + interface_name_ + " type can bitrate " + std::to_string(baudrate);
  int ret = system(cmd.c_str());
  return (ret == 0);
}

bool CanInterface::start()
{
  if (socket_fd_ < 0) {
    if (!initialize()) {
      return false;
    }
  }

  running_ = true;
  read_thread_ = std::thread(&CanInterface::read_loop, this);
  return true;
}

void CanInterface::stop()
{
  running_ = false;

  if (read_thread_.joinable()) {
    read_thread_.join();
  }

  if (socket_fd_ >= 0) {
    close(socket_fd_);
    socket_fd_ = -1;
  }

  release_interface_lock();
}

bool CanInterface::is_running() const { return running_; }

bool CanInterface::send_message(uint32_t id, const uint8_t * data, uint8_t dlc)
{
  if (socket_fd_ < 0) {
    return false;
  }

  struct can_frame frame;
  memset(&frame, 0, sizeof(frame));
  frame.can_id = id;
  frame.can_dlc = dlc;
  memcpy(frame.data, data, dlc);

  ssize_t nbytes = write(socket_fd_, &frame, sizeof(struct can_frame));
  if (nbytes != sizeof(struct can_frame)) {
    std::cerr << "Error sending CAN message: " << strerror(errno) << std::endl;
    return false;
  }
  return true;
}

bool CanInterface::send_message(const CanFrameMsg & msg)
{
  return send_message(msg.arbitration_id, msg.data, msg.dlc);
}

void CanInterface::read_loop()
{
  struct can_frame frame;
  CanFrameMsg msg;

  while (running_) {
    ssize_t nbytes = read(socket_fd_, &frame, sizeof(struct can_frame));

    if (nbytes == sizeof(struct can_frame)) {
      msg.arbitration_id = frame.can_id;
      msg.is_extended_id = (frame.can_id & CAN_EFF_FLAG) != 0;
      msg.dlc = frame.can_dlc;
      memcpy(msg.data, frame.data, frame.can_dlc);

      if (callback_) {
        callback_(msg);
      }
    } else if (nbytes == -1) {
      if (errno != EAGAIN && errno != EWOULDBLOCK) {
        std::cerr << "Error reading from CAN socket: " << strerror(errno) << std::endl;
      }
      // Sleep a bit to prevent CPU hogging
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }
}

void CanInterface::release_interface_lock()
{
  if (lock_fd_ >= 0) {
    // If we're releasing the lock, also remove the ready file
    std::string ready_file = "/var/lock/can_" + interface_name_ + ".ready";
    unlink(ready_file.c_str());

    flock(lock_fd_, LOCK_UN);
    close(lock_fd_);
    lock_fd_ = -1;
  }
}

}  // namespace piper
}  // namespace agilex