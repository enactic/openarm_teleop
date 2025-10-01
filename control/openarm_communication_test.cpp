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

#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <openarm/can/socket/openarm.hpp>
#include <openarm/damiao_motor/dm_motor_constants.hpp>
#include <thread>

int main(int argc, char** argv) {
    try {
        std::cout << "=== OpenArm CAN Example ===" << std::endl;
        std::cout << "This example demonstrates the OpenArm API functionality" << std::endl;

        std::string can_interface = "can0";
        if (argc > 1) {
            can_interface = argv[1];
        }

        std::cout << "[INFO] Using CAN interface: " << can_interface << std::endl;

        // Initialize OpenArm with CAN interface and enable CAN-FD
        std::cout << "Initializing OpenArm CAN..." << std::endl;
        openarm::can::socket::OpenArm openarm(can_interface, true);  // Use CAN-FD on can0 interface

        // Initialize arm motors
        std::vector<openarm::damiao_motor::MotorType> motor_types = {
            openarm::damiao_motor::MotorType::DM8009, openarm::damiao_motor::MotorType::DM8009,
            openarm::damiao_motor::MotorType::DM4340, openarm::damiao_motor::MotorType::DM4340,
            openarm::damiao_motor::MotorType::DM4310, openarm::damiao_motor::MotorType::DM4310,
            openarm::damiao_motor::MotorType::DM4310};

        std::vector<uint32_t> send_can_ids = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07};
        std::vector<uint32_t> recv_can_ids = {0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17};
        openarm.init_arm_motors(motor_types, send_can_ids, recv_can_ids);

        // Initialize gripper
        std::cout << "Initializing gripper..." << std::endl;
        openarm.init_gripper_motor(openarm::damiao_motor::MotorType::DM4310, 0x08, 0x18);

        // Set callback mode to ignore and refresh all motors
        openarm.set_callback_mode_all(openarm::damiao_motor::CallbackMode::IGNORE);
        openarm.refresh_all();
        openarm.recv_all();

        // Enable all motors
        std::cout << "\n=== Enabling Motors ===" << std::endl;
        openarm.enable_all();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        openarm.recv_all();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        // Set device mode to param and query motor id
        std::cout << "\n=== Querying Motor IDs ===" << std::endl;
        openarm.set_callback_mode_all(openarm::damiao_motor::CallbackMode::PARAM);
        openarm.query_param_all(static_cast<int>(openarm::damiao_motor::RID::MST_ID));
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        openarm.recv_all();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        // Access motors through components
        for (const auto& motor : openarm.get_arm().get_motors()) {
            std::cout << "Arm Motor: " << motor.get_send_can_id() << " ID: "
                      << motor.get_param(static_cast<int>(openarm::damiao_motor::RID::MST_ID))
                      << std::endl;
        }
        for (const auto& motor : openarm.get_gripper().get_motors()) {
            std::cout << "Gripper Motor: " << motor.get_send_can_id() << " ID: "
                      << motor.get_param(static_cast<int>(openarm::damiao_motor::RID::MST_ID))
                      << std::endl;
        }

        // Set device mode to state and control motor
        std::cout << "\n=== Controlling Motors ===" << std::endl;
        openarm.set_callback_mode_all(openarm::damiao_motor::CallbackMode::STATE);

        // Control arm motors
        openarm.get_arm().mit_control_all({openarm::damiao_motor::MITParam{0, 0, 0, 0, 0},
                                           openarm::damiao_motor::MITParam{0, 0, 0, 0, 0},
                                           openarm::damiao_motor::MITParam{0, 0, 0, 0, 0},
                                           openarm::damiao_motor::MITParam{0, 0, 0, 0, 0},
                                           openarm::damiao_motor::MITParam{0, 0, 0, 0, 0},
                                           openarm::damiao_motor::MITParam{0, 0, 0, 0, 0},
                                           openarm::damiao_motor::MITParam{0, 0, 0, 0, 0}});

        openarm.get_gripper().mit_control_all({openarm::damiao_motor::MITParam{0, 0, 0, 0, 0}});

        openarm.recv_all();

        // Control gripper
        std::cout << "Opening gripper..." << std::endl;
        // openarm.get_gripper().open();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        for (int i = 0; i < 100; i++) {
            openarm.refresh_all();
            openarm.recv_all();

            // Display arm motor states
            for (const auto& motor : openarm.get_arm().get_motors()) {
                std::cout << "Arm Motor: " << motor.get_send_can_id()
                          << " position: " << motor.get_position() << std::endl;
            }
            // Display gripper state
            for (const auto& motor : openarm.get_gripper().get_motors()) {
                std::cout << "Gripper Motor: " << motor.get_send_can_id()
                          << " position: " << motor.get_position() << std::endl;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        // Test gripper close
        std::cout << "Closing gripper..." << std::endl;
        // openarm.get_gripper().close();
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        openarm.disable_all();
        openarm.recv_all();

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return -1;
    }

    return 0;
}
