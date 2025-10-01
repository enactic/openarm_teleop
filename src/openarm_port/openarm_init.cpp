// Copyright 2025 Enactic, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "openarm_init.hpp"

#include "../openarm_constants.hpp"

namespace openarm_init {

openarm::can::socket::OpenArm *OpenArmInitializer::initialize_openarm(const std::string &can_device,
                                                                      bool enable_debug) {
    MotorConfig config = DEFAULT_MOTOR_CONFIG;
    return initialize_openarm(can_device, config, enable_debug);
}

openarm::can::socket::OpenArm *OpenArmInitializer::initialize_openarm(const std::string &can_device,
                                                                      const MotorConfig &config,
                                                                      bool enable_debug) {
    // Create OpenArm instance
    openarm::can::socket::OpenArm *openarm =
        new openarm::can::socket::OpenArm(can_device, enable_debug);

    // Perform common initialization
    initialize_(openarm, config, enable_debug);

    return openarm;
}

void OpenArmInitializer::initialize_(openarm::can::socket::OpenArm *openarm,
                                     const MotorConfig &config, bool enable_debug) {
    if (enable_debug) {
        std::cout << "Initializing arm motors..." << std::endl;
    }

    // Initialize arm motors
    openarm->init_arm_motors(config.arm_motor_types, config.arm_send_can_ids,
                             config.arm_recv_can_ids);

    if (enable_debug) {
        std::cout << "Initializing gripper motor..." << std::endl;
    }

    // Initialize gripper motor
    openarm->init_gripper_motor(config.gripper_motor_type, config.gripper_send_can_id,
                                config.gripper_recv_can_id);

    // Set callback mode for all motors
    openarm->set_callback_mode_all(openarm::damiao_motor::CallbackMode::STATE);

    if (enable_debug) {
        std::cout << "Enabling motors..." << std::endl;
    }

    // Enable all motors with appropriate delays
    openarm->enable_all();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    openarm->recv_all();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Print motor counts for verification
    if (enable_debug) {
        size_t arm_motor_num = openarm->get_arm().get_motors().size();
        size_t gripper_motor_num = openarm->get_gripper().get_motors().size();

        std::cout << "Arm motor count: " << arm_motor_num << std::endl;
        std::cout << "Gripper motor count: " << gripper_motor_num << std::endl;
    }
}

}  // namespace openarm_init
