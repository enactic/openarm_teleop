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

#include "joint_mapper.hpp"

#include <cmath>

JointMapper::JointMapper() {}

JointMapper::~JointMapper() {}

// Only copying for now
void JointMapper::motor_to_joint_position(const double *motor_position, double *joint_position) {
    joint_position[0] = motor_position[0];
    joint_position[1] = motor_position[1];
    joint_position[2] = motor_position[2];
    joint_position[3] = motor_position[3];
    joint_position[4] = motor_position[4];
    joint_position[5] = motor_position[5];
    joint_position[6] = motor_position[6];
    joint_position[7] = motor_position[7];
}

void JointMapper::motor_to_joint_velocity(const double *motor_velocity, double *joint_velocity) {
    joint_velocity[0] = motor_velocity[0];
    joint_velocity[1] = motor_velocity[1];
    joint_velocity[2] = motor_velocity[2];
    joint_velocity[3] = motor_velocity[3];
    joint_velocity[4] = motor_velocity[4];
    joint_velocity[5] = motor_velocity[5];
    joint_velocity[6] = motor_velocity[6];
    joint_velocity[7] = motor_velocity[7];
}

void JointMapper::joint_to_motor_torque(const double *joint_torque, double *motor_torque) {
    motor_torque[0] = joint_torque[0];
    motor_torque[1] = joint_torque[1];
    motor_torque[2] = joint_torque[2];
    motor_torque[3] = joint_torque[3];
    motor_torque[4] = joint_torque[4];
    motor_torque[5] = joint_torque[5];
    motor_torque[6] = joint_torque[6];
    motor_torque[7] = joint_torque[7];
}
