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

#include <mutex>
#include <vector>

// Represents the state of a single joint: position, velocity, and effort.
struct JointState {
    double position = 0.0;
    double velocity = 0.0;
    double effort = 0.0;
};

// Manages reference and response states for a robot component (e.g., arm, hand).
class RobotState {
public:
    RobotState() = default;

    explicit RobotState(size_t num_joints) : response_(num_joints), reference_(num_joints) {}

    // --- Set/Get reference (target) joint states ---
    void set_reference(size_t index, const JointState& state) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (index < reference_.size()) {
            reference_[index] = state;
        }
    }

    void set_all_references(const std::vector<JointState>& states) {
        std::lock_guard<std::mutex> lock(mutex_);
        reference_ = states;
    }

    JointState get_reference(size_t index) const {
        std::lock_guard<std::mutex> lock(mutex_);
        return index < reference_.size() ? reference_[index] : JointState{};
    }

    std::vector<JointState> get_all_references() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return reference_;
    }

    void set_response(size_t index, const JointState& state) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (index < response_.size()) {
            response_[index] = state;
        }
    }

    void set_all_responses(const std::vector<JointState>& states) {
        std::lock_guard<std::mutex> lock(mutex_);
        response_ = states;
    }

    JointState get_response(size_t index) const {
        std::lock_guard<std::mutex> lock(mutex_);
        return index < response_.size() ? response_[index] : JointState{};
    }

    std::vector<JointState> get_all_responses() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return response_;
    }

    void resize(size_t new_size) {
        std::lock_guard<std::mutex> lock(mutex_);
        reference_.resize(new_size);
        response_.resize(new_size);
    }

    size_t get_size() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return response_.size();  // assume same size for both
    }

private:
    mutable std::mutex mutex_;
    std::vector<JointState> response_;
    std::vector<JointState> reference_;
};

// Manages the joint states of robot components (arm, hand).
class RobotSystemState {
public:
    RobotSystemState(size_t arm_joint_count, size_t hand_joint_count)
        : arm_state_(arm_joint_count), hand_state_(hand_joint_count) {}

    RobotState& arm_state() { return arm_state_; }
    RobotState& hand_state() { return hand_state_; }

    const RobotState& arm_state() const { return arm_state_; }
    const RobotState& hand_state() const { return hand_state_; }

    std::vector<JointState> get_all_responses() const {
        auto arm = arm_state_.get_all_responses();
        auto hand = hand_state_.get_all_responses();
        std::vector<JointState> combined;
        combined.reserve(arm.size() + hand.size());
        combined.insert(combined.end(), arm.begin(), arm.end());
        combined.insert(combined.end(), hand.begin(), hand.end());
        return combined;
    }

    void set_all_references(const std::vector<JointState>& all_refs) {
        const size_t arm_size = arm_state_.get_size();
        const size_t hand_size = hand_state_.get_size();

        if (all_refs.size() != arm_size + hand_size) {
            throw std::runtime_error("set_all_references: size mismatch.");
        }

        std::vector<JointState> arm_refs(all_refs.begin(), all_refs.begin() + arm_size);
        std::vector<JointState> hand_refs(all_refs.begin() + arm_size, all_refs.end());

        arm_state_.set_all_references(arm_refs);
        hand_state_.set_all_references(hand_refs);
    }

    std::vector<JointState> get_all_references() const {
        auto arm = arm_state_.get_all_references();
        auto hand = hand_state_.get_all_references();
        std::vector<JointState> combined;
        combined.reserve(arm.size() + hand.size());
        combined.insert(combined.end(), arm.begin(), arm.end());
        combined.insert(combined.end(), hand.begin(), hand.end());
        return combined;
    }

    void set_all_responses(const std::vector<JointState>& all_responses) {
        const size_t arm_size = arm_state_.get_size();
        const size_t hand_size = hand_state_.get_size();

        std::cout << "arm_size : " << arm_size << std::endl;
        std::cout << "hand_size : " << hand_size << std::endl;
        std::cout << "all_responses.size() : " << all_responses.size() << std::endl;

        if (all_responses.size() != arm_size + hand_size) {
            throw std::runtime_error("set_all_responses: size mismatch.");
        }

        std::vector<JointState> arm_res(all_responses.begin(), all_responses.begin() + arm_size);
        std::vector<JointState> hand_res(all_responses.begin() + arm_size, all_responses.end());

        arm_state_.set_all_responses(arm_res);
        hand_state_.set_all_responses(hand_res);
    }

    size_t get_total_joint_count() const { return arm_state_.get_size() + hand_state_.get_size(); }

private:
    RobotState arm_state_;
    RobotState hand_state_;
};
