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
// #include "controller/global.hpp"
// #include <global.hpp>

#include <openarm_constants.hpp>

class Differentiator {
private:
    double Ts_;                            // Sampling time
    double velocity_z1_[NMOTORS] = {0.0};  // Velocity (1 step before)
    double position_z1_[NMOTORS] = {0.0};  // Position (1 step before)
    double acc_z1_[NMOTORS] = {0.0};
    double acc_[NMOTORS] = {0.0};

public:
    Differentiator(double Ts) : Ts_(Ts) {}

    /*
     * Compute the motor speed by taking the derivative of
     * the motion.
     */
    void Differentiate(const double *position, double *velocity) {
        double a = 1.0 / (1.0 + Ts_ * CUTOFF_FREQUENCY);
        double b = a * CUTOFF_FREQUENCY;

        for (int i = 0; i < NMOTORS; i++) {
            if (position_z1_[i] == 0.0) {
                position_z1_[i] = position[i];
            }

            velocity[i] = velocity_z1_[i] * a + b * (position[i] - position_z1_[i]);
            position_z1_[i] = position[i];
            velocity_z1_[i] = velocity[i];
        }
    }

    void Differentiate_w_obs(const double *position, double *velocity, double *mass,
                             double *input_torque) {
        double a = 1.0 / (1.0 + Ts_ * CUTOFF_FREQUENCY);
        double b = a * CUTOFF_FREQUENCY;

        for (int i = 0; i < NMOTORS; i++) {
            if (position_z1_[i] == 0.0000000) {
                position_z1_[i] = position[i];
                acc_z1_[i] = acc_[i];
            }

            acc_[i] = acc_z1_[i] * a + b * (input_torque[i] / (mass[i]));
            velocity[i] = velocity_z1_[i] * a + b * (position[i] - position_z1_[i]) + acc_[i];
            position_z1_[i] = position[i];
            velocity_z1_[i] = velocity[i];
            acc_z1_[i] = acc_[i];
        }
    }
};
