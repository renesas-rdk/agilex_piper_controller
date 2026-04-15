#include "agilex_piper_controller/piper_controller.hpp"

int main() {
    // Initialize controller with CAN interface name
    agilex::piper::PiperController controller("piper0");

    // Enable the arm
    controller.enable_arm();

    // Set control mode (CAN command control mode, joint movement)
    controller.set_mode(0x01, 0x01, 50);  // Mode, move type, speed rate

    // Move to a joint position
    controller.set_joint_angles(0, 0, 0, 0, 0, 0);  // All joints to zero

    return 0;
}