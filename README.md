# Agilex Piper Controller

## Overview

This package provides a C++ implementation of the controller for the Agilex Piper robotic arm. It is a port of the original [Python piper_sdk](https://github.com/agilexrobotics/piper_sdk), simplified to include only the essential command interfaces needed for controlling the robot.

## Features

- **C++ Native Implementation**: Complete rewrite of the Python SDK in C++
- **Pure CMake Project**: No ROS runtime/build dependency required
- **CAN Communication**: Direct CAN bus communication with the Piper arm
- **Simplified API**: Focuses on core functionality for easier integration

## Architecture

The controller is structured around several key components:

- **PiperController**: Main class for interfacing with the robot
- **PiperProtocol**: Handles message encoding and decoding
- **CanInterface**: Manages low-level CAN communication
- **Message Definitions**: Complete set of command and feedback message structures

## Installation

### Prerequisites

- Linux with socketCAN support
- C++17 compiler
- CMake (>= 3.8)
- Eigen3

### Building

```bash
# Configure
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release

# Build
cmake --build build --parallel
```

### Building with Pixi

```bash
# Configure once
pixi run configure

# Build
pixi run build

# Install to .pixi/install
pixi run install
```

Useful commands:

```bash
# Run tests (if/when tests are added)
pixi run test

# Remove build artifacts created by Pixi tasks
pixi run clean
```

## Usage

### Basic Example

```cpp
#include "agilex_piper_controller/piper_controller.hpp"

int main() {
    // Initialize controller with CAN interface name
    agilex::piper::PiperController controller("can0");

    // Enable the arm
    controller.enable_arm();

    // Set control mode (CAN command control mode, joint movement)
    controller.set_mode(0x01, 0x01, 50);  // Mode, move type, speed rate

    // Move to a joint position
    controller.set_joint_angles(0, 0, 0, 0, 0, 0);  // All joints to zero

    return 0;
}
```

### Key Methods

- `connect_port()`: Establishes connection to the CAN bus
- `enable_arm()` / `disable_arm()`: Controls power to the arm motors
- `set_mode()`: Sets control and movement modes
- `set_joint_angles()`: Positions the arm using joint angles
- `set_end_pose()`: Positions the arm using cartesian coordinates
- `control_gripper()`: Controls the gripper position and force

## Differences from Original SDK

This implementation is a simplified version of the original Python SDK:

1. **Focused Functionality**: Only includes essential command interfaces
2. **No GUI Components**: Removed visualization and GUI tools
3. **Streamlined Protocol**: Simplified the communication protocol
4. **C++ Performance**: Optimized for performance with C++ implementation

## License

This package is released under the Apache License 2.0, the same license as the original piper_sdk.

## Acknowledgments

This work is based on the [piper_sdk](https://github.com/agilexrobotics/piper_sdk) from Agilex Robotics. The original implementation was in Python, and this package represents a C++ port of that functionality.
