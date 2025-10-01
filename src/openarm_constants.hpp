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

#pragma once

#include <time.h>
#include <unistd.h>

#include <iostream>
#include <openarm/damiao_motor/dm_motor_constants.hpp>
#include <vector>

constexpr double PI = 3.14159265358979323846;

// 8piecies including gripper
// Joints and motors don't always have a one-to-one correspondence
#define NJOINTS 8
#define NMOTORS 8

#define ROLE_LEADER 1
#define ROLE_FOLLOWER 2

#define CAN0 "can0"
#define CAN1 "can1"

#define CAN2 "can2"
#define CAN3 "can3"

#define TANHFRIC true

#define FREQUENCY 1000.0
#define CUTOFF_FREQUENCY 90.0

#define ELBOWLIMIT 0.0

static const double INITIAL_POSITION[NMOTORS] = {0, 0, 0, PI / 5.0, 0, 0, 0, 0};

// safety limit position
static const double position_limit_max_L[] = {(2.0 / 3.0) * PI, PI,       PI / 2.0, PI,
                                              PI / 2.0,         PI / 2.0, PI / 2.0, PI};
static const double position_limit_min_L[] = {-(2.0 / 3.0) * PI, -PI / 2.0, -PI / 2.0, ELBOWLIMIT,
                                              -PI / 2.0,         -PI / 2.0, -PI / 2.0, -PI};
static const double position_limit_max_F[] = {(2.0 / 3.0) * PI, PI,       PI / 2.0, PI,
                                              PI / 2.0,         PI / 2.0, PI / 2.0, PI};
static const double position_limit_min_F[] = {-(2.0 / 3.0) * PI, -PI / 2.0, -PI / 2.0, ELBOWLIMIT,
                                              -PI / 2.0,         -PI / 2.0, -PI / 2.0, -PI};

// sefaty limit velocity
static const double velocity_limit_L[] = {8.0, 8.0, 8.0, 8.0, 8.0, 8.0, 8.0, 8.0};
static const double velocity_limit_F[] = {8.0, 8.0, 8.0, 8.0, 8.0, 8.0, 8.0, 8.0};
// sefaty limit effort
static const double effort_limit_L[] = {20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0};
static const double effort_limit_F[] = {20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0};

// Motor configuration structure
struct MotorConfig {
    std::vector<openarm::damiao_motor::MotorType> arm_motor_types;
    std::vector<uint32_t> arm_send_can_ids;
    std::vector<uint32_t> arm_recv_can_ids;
    openarm::damiao_motor::MotorType gripper_motor_type;
    uint32_t gripper_send_can_id;
    uint32_t gripper_recv_can_id;
};

// Global default motor configuration
static const MotorConfig DEFAULT_MOTOR_CONFIG = {
    // Standard 7-DOF arm motor configuration
    {openarm::damiao_motor::MotorType::DM8009, openarm::damiao_motor::MotorType::DM8009,
     openarm::damiao_motor::MotorType::DM4340, openarm::damiao_motor::MotorType::DM4340,
     openarm::damiao_motor::MotorType::DM4310, openarm::damiao_motor::MotorType::DM4310,
     openarm::damiao_motor::MotorType::DM4310},

    // Standard CAN IDs for arm motors
    {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07},
    {0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17},

    // Standard gripper configuration
    openarm::damiao_motor::MotorType::DM4310,
    0x08,
    0x18};

// opening function
inline void printOpenArmBanner() {
    std::cout << R"(

                                     ██████╗ ██████╗ ███████╗███╗   ██╗ █████╗ ██████╗ ███╗   ███╗
                                    ██╔═══██╗██╔══██╗██╔════╝████╗  ██║██╔══██╗██╔══██╗████╗ ████║
                                    ██║   ██║██████╔╝█████╗  ██╔██╗ ██║███████║██████╔╝██╔████╔██║
                                    ██║   ██║██╔═══╝ ██╔══╝  ██║╚██╗██║██╔══██║██╔══██╗██║╚██╔╝██║
                                    ╚██████╔╝██║     ███████╗██║ ╚████║██║  ██║██║  ██║██║ ╚═╝ ██║
                                     ╚═════╝ ╚═╝     ╚══════╝╚═╝  ╚═══╝╚═╝  ╚═╝╚═╝  ╚═╝╚═╝     ╚═╝

██████╗ ██╗██╗      █████╗ ████████╗███████╗██████╗  █████╗ ██╗          ██████╗ ██████╗ ███╗   ██╗████████╗██████╗  ██████╗ ██╗     ██╗██╗██╗██╗
██╔══██╗██║██║     ██╔══██╗╚══██╔══╝██╔════╝██╔══██╗██╔══██╗██║         ██╔════╝██╔═══██╗████╗  ██║╚══██╔══╝██╔══██╗██╔═══██╗██║     ██║██║██║██║
██████╔╝██║██║     ███████║   ██║   █████╗  ██████╔╝███████║██║         ██║     ██║   ██║██╔██╗ ██║   ██║   ██████╔╝██║   ██║██║     ██║██║██║██║
██╔══██╗██║██║     ██╔══██║   ██║   ██╔══╝  ██╔══██╗██╔══██║██║         ██║     ██║   ██║██║╚██╗██║   ██║   ██╔══██╗██║   ██║██║     ╚═╝╚═╝╚═╝╚═╝
██████╔╝██║███████╗██║  ██║   ██║   ███████╗██║  ██║██║  ██║███████╗    ╚██████╗╚██████╔╝██║ ╚████║   ██║   ██║  ██║╚██████╔╝███████╗██╗██╗██╗██╗
╚═════╝ ╚═╝╚══════╝╚═╝  ╚═╝   ╚═╝   ╚══════╝╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝     ╚═════╝ ╚═════╝ ╚═╝  ╚═══╝   ╚═╝   ╚═╝  ╚═╝ ╚═════╝ ╚══════╝╚═╝╚═╝╚═╝╚═╝

    )" << std::endl;
}
