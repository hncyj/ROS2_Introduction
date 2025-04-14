#include "../include/motion_control_system/spin_motion_controller.hpp"
#include <iostream>

namespace motion_control_system {
    void SpinMotionController::start() {
        std::cout << "Spin motion controller started." << std::endl;
    }

    void SpinMotionController::stop() {
        std::cout << "Spin motion controller stopped." << std::endl;
    }
}

#include "pluginlib/class_list_macros.hpp"

// This macro is used to register the SpinMotionController class as a plugin
// with the pluginlib library. The first argument is the class name, and the
// second argument is the base class name.
PLUGINLIB_EXPORT_CLASS(motion_control_system::SpinMotionController, motion_control_system::MotionController)
