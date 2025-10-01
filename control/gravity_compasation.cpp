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
#include <controller/dynamics.hpp>
#include <csignal>
#include <filesystem>
#include <iostream>
#include <openarm/can/socket/openarm.hpp>
#include <openarm/damiao_motor/dm_motor_constants.hpp>
#include <openarm_port/openarm_init.hpp>
#include <thread>

std::atomic<bool> keep_running(true);

void signal_handler(int signal) {
    if (signal == SIGINT) {
        std::cout << "\nCtrl+C detected. Exiting loop..." << std::endl;
        keep_running = false;
    }
}

int main(int argc, char** argv) {
    try {
        std::signal(SIGINT, signal_handler);

        std::string arm_side = "right_arm";
        std::string can_interface = "can0";

        if (argc < 4) {
            std::cerr << "Usage: " << argv[0] << " <arm_side> <can_interface> <urdf_path>"
                      << std::endl;
            std::cerr << "Example: " << argv[0] << " right_arm can0 /tmp/v10_bimanual.urdf"
                      << std::endl;
            return 1;
        }

        arm_side = argv[1];
        can_interface = argv[2];
        std::string urdf_path = argv[3];

        if (arm_side != "left_arm" && arm_side != "right_arm") {
            std::cerr << "[ERROR] Invalid arm_side: " << arm_side
                      << ". Must be 'left_arm' or 'right_arm'." << std::endl;
            return 1;
        }

        if (!std::filesystem::exists(urdf_path)) {
            std::cerr << "[ERROR] URDF file not found: " << urdf_path << std::endl;
            return 1;
        }

        std::cout << "=== OpenArm Gravity Compensation ===" << std::endl;
        std::cout << "Arm side       : " << arm_side << std::endl;
        std::cout << "CAN interface  : " << can_interface << std::endl;
        std::cout << "URDF path      : " << urdf_path << std::endl;

        std::string root_link = "openarm_body_link0";
        std::string leaf_link =
            (arm_side == "left_arm") ? "openarm_left_hand" : "openarm_right_hand";

        Dynamics arm_dynamics(urdf_path, root_link, leaf_link);
        arm_dynamics.Init();

        std::cout << "=== Initializing Leader OpenArm ===" << std::endl;
        openarm::can::socket::OpenArm* openarm =
            openarm_init::OpenArmInitializer::initialize_openarm(can_interface, true);

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        auto start_time = std::chrono::high_resolution_clock::now();
        auto last_hz_display = start_time;
        int frame_count = 0;

        std::vector<double> arm_joint_positions(openarm->get_arm().get_motors().size(), 0.0);
        std::vector<double> arm_joint_velocities(openarm->get_arm().get_motors().size(), 0.0);

        std::vector<double> gripper_joint_positions(openarm->get_gripper().get_motors().size(),
                                                    0.0);
        std::vector<double> gripper_joint_velocities(openarm->get_gripper().get_motors().size(),
                                                     0.0);

        std::vector<double> grav_torques(openarm->get_arm().get_motors().size(), 0.0);

        while (keep_running) {
            frame_count++;
            auto current_time = std::chrono::high_resolution_clock::now();

            // Calculate and display Hz every second
            auto time_since_last_display = std::chrono::duration_cast<std::chrono::milliseconds>(
                                               current_time - last_hz_display)
                                               .count();
            if (time_since_last_display >= 1000) {  // Every 1000ms (1 second)
                auto total_time =
                    std::chrono::duration_cast<std::chrono::milliseconds>(current_time - start_time)
                        .count();
                double hz = (frame_count * 1000.0) / total_time;
                std::cout << "=== Loop Frequency: " << hz << " Hz ===" << std::endl;
                last_hz_display = current_time;
            }

            auto motors = openarm->get_arm().get_motors();
            for (size_t i = 0; i < motors.size(); ++i) {
                arm_joint_positions[i] = motors[i].get_position();
                arm_joint_velocities[i] = motors[i].get_velocity();
            }

            arm_dynamics.GetGravity(arm_joint_positions.data(), grav_torques.data());

            for (size_t i = 0; i < openarm->get_arm().get_motors().size(); ++i) {
                // std::cout << "grav_torques[" << i << "] = " << grav_torques[i] << std::endl;
            }

            std::vector<openarm::damiao_motor::MITParam> cmds;
            cmds.reserve(grav_torques.size());

            std::transform(grav_torques.begin(), grav_torques.end(), std::back_inserter(cmds),
                           [](double t) { return openarm::damiao_motor::MITParam{0, 0, 0, 0, t}; });

            openarm->get_arm().mit_control_all(cmds);

            openarm->recv_all();
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        openarm->disable_all();
        openarm->recv_all();

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return -1;
    }

    return 0;
}
