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

#include "../openarm_constants.hpp"

class JointMapper {
public:
    JointMapper();
    ~JointMapper();

    void motor_to_joint_position(const double *motor_position, double *joint_position);
    void motor_to_joint_velocity(const double *motor_velocity, double *joint_velocity);
    void joint_to_motor_torque(const double *joint_torque, double *motor_torque);
};
