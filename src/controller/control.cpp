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

#include <string.h>
#include <unistd.h>

#include <algorithm>
#include <cmath>
#include <controller/control.hpp>
#include <controller/dynamics.hpp>
#include <iomanip>
#include <thread>

Control::Control(openarm::can::socket::OpenArm* arm, Dynamics* dynamics_l, Dynamics* dynamics_f,
                 std::shared_ptr<RobotSystemState> robot_state, double Ts, int role,
                 size_t arm_motor_num, size_t hand_motor_num)
    : openarm_(arm),
      dynamics_l_(dynamics_l),
      dynamics_f_(dynamics_f),
      robot_state_(robot_state),
      Ts_(Ts),
      role_(role),
      arm_motor_num_(arm_motor_num),
      hand_motor_num_(hand_motor_num) {
    differentiator_ = new Differentiator(Ts);
    openarmjointconverter_ = new OpenArmJointConverter(arm_motor_num_);
    openarmgripperjointconverter_ = new OpenArmJGripperJointConverter(hand_motor_num_);
}

Control::Control(openarm::can::socket::OpenArm* arm, Dynamics* dynamics_l, Dynamics* dynamics_f,
                 std::shared_ptr<RobotSystemState> robot_state, double Ts, int role,
                 std::string arm_type, size_t arm_motor_num, size_t hand_motor_num)
    : openarm_(arm),
      dynamics_l_(dynamics_l),
      dynamics_f_(dynamics_f),
      robot_state_(robot_state),
      Ts_(Ts),
      role_(role),
      arm_motor_num_(arm_motor_num),
      hand_motor_num_(hand_motor_num) {
    differentiator_ = new Differentiator(Ts);
    openarmjointconverter_ = new OpenArmJointConverter(arm_motor_num_);
    openarmgripperjointconverter_ = new OpenArmJGripperJointConverter(hand_motor_num_);

    arm_type_ = arm_type;
}

Control::~Control() {
    std::cout << "Control destructed " << std::endl;
    delete openarmjointconverter_;
    delete differentiator_;
}

// bool Control::Setup(void)
// {
//         // double motor_position[NMOTORS] = {0.0};

//         // ComputeJointPosition(motor_position, response_->position.data());

//         std::cout << "!control->Setup()  finished "<< std::endl;

//         return true;
// }

void Control::Shutdown(void) {
    std::cout << "control shutdown !!!" << std::endl;

    openarm_->disable_all();
}

void Control::SetParameter(const std::vector<double>& Kp, const std::vector<double>& Kd,
                           const std::vector<double>& Fc, const std::vector<double>& k,
                           const std::vector<double>& Fv, const std::vector<double>& Fo) {
    Kp_ = Kp;
    Kd_ = Kd;
    Fc_ = Fc;
    k_ = k;
    Fv_ = Fv;
    Fo_ = Fo;
}

bool Control::bilateral_step() {
    // get motor status
    std::vector<MotorState> arm_motor_states;
    const auto& arm_motors = openarm_->get_arm().get_motors();
    for (size_t i = 0; i < arm_motors.size(); ++i) {
        const auto& motor = arm_motors[i];
        arm_motor_states.push_back({motor.get_position(), motor.get_velocity(), 0});
    }

    std::vector<MotorState> gripper_motor_states;
    const auto& gripper_motors = openarm_->get_gripper().get_motors();
    for (size_t i = 0; i < gripper_motors.size(); ++i) {
        const auto& motor = gripper_motors[i];
        gripper_motor_states.push_back({motor.get_position(), motor.get_velocity(), 0});
    }

    // convert joint to motor
    std::vector<JointState> joint_arm_states =
        openarmjointconverter_->motor_to_joint(arm_motor_states);
    std::vector<JointState> joint_gripper_states =
        openarmgripperjointconverter_->motor_to_joint(gripper_motor_states);

    // set reponse
    robot_state_->arm_state().set_all_responses(joint_arm_states);
    robot_state_->hand_state().set_all_responses(joint_gripper_states);

    size_t arm_dof = robot_state_->arm_state().get_size();
    size_t gripper_dof = robot_state_->hand_state().get_size();

    std::vector<double> joint_arm_positions(arm_dof, 0.0);
    std::vector<double> joint_arm_velocities(arm_dof, 0.0);
    std::vector<double> joint_arm_efforts(arm_dof, 0.0);

    std::vector<double> joint_gripper_positions(gripper_dof, 0.0);
    std::vector<double> joint_gripper_velocities(gripper_dof, 0.0);
    std::vector<double> joint_gripper_efforts(gripper_dof, 0.0);

    for (size_t i = 0; i < arm_dof; ++i) {
        joint_arm_positions[i] = joint_arm_states[i].position;
        joint_arm_velocities[i] = joint_arm_states[i].velocity;
    }

    for (size_t i = 0; i < gripper_dof; ++i) {
        joint_gripper_positions[i] = joint_gripper_states[i].position;
        joint_gripper_velocities[i] = joint_gripper_states[i].velocity;
    }

    std::vector<double> gravity(arm_dof, 0.0);
    std::vector<double> coriolis(arm_dof, 0.0);
    std::vector<double> friction(arm_dof + gripper_dof, 0.0);

    std::vector<JointState> joint_arm_states_ref = robot_state_->arm_state().get_all_references();
    std::vector<JointState> joint_gripper_states_ref =
        robot_state_->hand_state().get_all_references();

    std::vector<double> joint_arm_positions_ref(arm_dof);

    for (size_t i = 0; i < arm_dof; ++i) {
        joint_arm_positions_ref[i] = joint_arm_states_ref[i].position;
    }

    if (role_ == ROLE_LEADER) {
        dynamics_l_->GetGravity(joint_arm_positions.data(), gravity.data());
        dynamics_l_->GetCoriolis(joint_arm_positions.data(), joint_arm_velocities.data(),
                                 coriolis.data());

    } else if (role_ == ROLE_FOLLOWER) {
        dynamics_f_->GetGravity(joint_arm_positions.data(), gravity.data());
        dynamics_f_->GetCoriolis(joint_arm_positions.data(), joint_arm_velocities.data(),
                                 coriolis.data());
    }

    // Friction (compute joint friction)
    for (size_t i = 0; i < joint_arm_velocities.size(); ++i)
        ComputeFriction(joint_arm_velocities.data(), friction.data(), i);
    for (size_t i = 0; i < joint_gripper_velocities.size(); ++i)
        ComputeFriction(joint_gripper_velocities.data(), friction.data(),
                        joint_arm_velocities.size() + i);

    // set gravity and friciton comp joint torque value
    for (size_t i = 0; i < arm_dof; i++) {
        joint_arm_states_ref[i].effort = gravity[i] + friction[i];
    }

    for (size_t i = 0; i < gripper_dof; i++) {
        joint_gripper_states_ref[i].effort = friction[i + arm_dof];
    }

    std::vector<MotorState> motor_arm_states =
        openarmjointconverter_->joint_to_motor(joint_arm_states_ref);
    std::vector<MotorState> motor_gripper_states =
        openarmgripperjointconverter_->joint_to_motor(joint_gripper_states_ref);

    // kp kd q dq tau
    std::vector<openarm::damiao_motor::MITParam> arm_cmds;
    arm_cmds.reserve(arm_dof);
    for (size_t i = 0; i < arm_dof; ++i) {
        arm_cmds.emplace_back(openarm::damiao_motor::MITParam{
            Kp_[i], Kd_[i], motor_arm_states[i].position, motor_arm_states[i].velocity,
            motor_arm_states[i].effort});
    }

    // gripper command mit param
    std::vector<openarm::damiao_motor::MITParam> gripper_cmds;
    gripper_cmds.reserve(gripper_dof);
    for (size_t i = 0; i < gripper_dof; ++i) {
        gripper_cmds.emplace_back(openarm::damiao_motor::MITParam{
            Kp_[i + arm_dof], Kd_[i + arm_dof], motor_gripper_states[i].position,
            motor_gripper_states[i].velocity, motor_gripper_states[i].effort});
    }

    // send command to arm
    openarm_->get_arm().mit_control_all(arm_cmds);
    // send command to gripper
    openarm_->get_gripper().mit_control_all(gripper_cmds);

    std::this_thread::sleep_for(std::chrono::microseconds(200));

    openarm_->recv_all(220);

    return true;
}

bool Control::unilateral_step() {
    // get motor status
    std::vector<MotorState> arm_motor_states;
    for (const auto& motor : openarm_->get_arm().get_motors()) {
        arm_motor_states.push_back({motor.get_position(), motor.get_velocity(), 0.0});
    }

    std::vector<MotorState> gripper_motor_states;
    for (const auto& motor : openarm_->get_gripper().get_motors()) {
        gripper_motor_states.push_back({motor.get_position(), motor.get_velocity(), 0.0});
    }

    // convert joint to motor
    std::vector<JointState> joint_arm_states =
        openarmjointconverter_->motor_to_joint(arm_motor_states);
    std::vector<JointState> joint_gripper_states =
        openarmgripperjointconverter_->motor_to_joint(gripper_motor_states);

    // set reponse
    robot_state_->arm_state().set_all_responses(joint_arm_states);
    robot_state_->hand_state().set_all_responses(joint_gripper_states);

    size_t arm_dof = robot_state_->arm_state().get_size();
    size_t gripper_dof = robot_state_->hand_state().get_size();

    std::vector<double> joint_arm_positions(arm_dof, 0.0);
    std::vector<double> joint_arm_velocities(arm_dof, 0.0);
    std::vector<double> joint_gripper_positions(gripper_dof, 0.0);
    std::vector<double> joint_gripper_velocities(gripper_dof, 0.0);

    for (size_t i = 0; i < arm_dof; ++i) {
        joint_arm_positions[i] = joint_arm_states[i].position;
        joint_arm_velocities[i] = joint_arm_states[i].velocity;
    }

    for (size_t i = 0; i < gripper_dof; ++i) {
        joint_gripper_positions[i] = joint_gripper_states[i].position;
        joint_gripper_velocities[i] = joint_gripper_states[i].velocity;
    }

    std::vector<double> gravity(arm_dof, 0.0);
    std::vector<double> coriolis(arm_dof, 0.0);
    std::vector<double> friction(arm_dof + gripper_dof, 0.0);

    if (role_ == ROLE_LEADER) {
        // calc dynamics
        dynamics_l_->GetGravity(joint_arm_positions.data(), gravity.data());
        dynamics_l_->GetCoriolis(joint_arm_positions.data(), joint_arm_velocities.data(),
                                 coriolis.data());

        for (size_t i = 0; i < joint_arm_velocities.size(); ++i)
            ComputeFriction(joint_arm_velocities.data(), friction.data(), i);

        for (size_t i = 0; i < joint_gripper_velocities.size(); ++i)
            ComputeFriction(joint_gripper_velocities.data(), friction.data(), arm_dof + i);

        // arm joint state
        std::vector<JointState> joint_arm_state_torque(arm_dof);
        for (size_t i = 0; i < arm_dof; ++i) {
            joint_arm_state_torque[i].position = joint_arm_positions[i];
            joint_arm_state_torque[i].velocity = joint_arm_velocities[i];
            joint_arm_state_torque[i].effort = gravity[i] + friction[i] * 0.3 + coriolis[i] * 0.1;
        }

        // gripper joint state
        std::vector<JointState> joint_gripper_state_torque(gripper_dof);
        for (size_t i = 0; i < gripper_dof; ++i) {
            joint_gripper_state_torque[i].position = joint_gripper_positions[i];
            joint_gripper_state_torque[i].velocity = joint_gripper_velocities[i];
            joint_gripper_state_torque[i].effort = friction[arm_dof + i] * 0.3;
        }

        std::vector<MotorState> motor_arm_states =
            openarmjointconverter_->joint_to_motor(joint_arm_state_torque);
        std::vector<MotorState> motor_gripper_states =
            openarmgripperjointconverter_->joint_to_motor(joint_gripper_state_torque);

        // arm command mit param
        std::vector<openarm::damiao_motor::MITParam> arm_cmds;
        arm_cmds.reserve(arm_dof);
        for (size_t i = 0; i < arm_dof; ++i) {
            arm_cmds.emplace_back(
                openarm::damiao_motor::MITParam{0.0, 0.0, 0.0, 0.0, motor_arm_states[i].effort});
        }

        // gripper command mit param
        std::vector<openarm::damiao_motor::MITParam> gripper_cmds;
        gripper_cmds.reserve(gripper_dof);
        for (size_t i = 0; i < gripper_dof; ++i) {
            gripper_cmds.emplace_back(openarm::damiao_motor::MITParam{
                0.0, 0.0, 0.0, 0.0, motor_gripper_states[i].effort});
        }

        // send command to arm
        openarm_->get_arm().mit_control_all(arm_cmds);
        // send command to gripper
        openarm_->get_gripper().mit_control_all(gripper_cmds);

        openarm_->recv_all(200);

        return true;

    }

    else if (role_ == ROLE_FOLLOWER) {
        std::vector<JointState> joint_arm_states_ref =
            robot_state_->arm_state().get_all_references();
        std::vector<JointState> joint_hand_states_ref =
            robot_state_->hand_state().get_all_references();

        // Joint → Motor
        std::vector<MotorState> arm_motor_refs =
            openarmjointconverter_->joint_to_motor(joint_arm_states_ref);
        std::vector<MotorState> hand_motor_refs =
            openarmgripperjointconverter_->joint_to_motor(joint_hand_states_ref);

        std::vector<openarm::damiao_motor::MITParam> arm_cmds;
        arm_cmds.reserve(arm_motor_refs.size());
        for (size_t i = 0; i < arm_motor_refs.size(); ++i) {
            arm_cmds.emplace_back(openarm::damiao_motor::MITParam{
                Kp_[i], Kd_[i], arm_motor_refs[i].position, arm_motor_refs[i].velocity, 0.0});
        }

        std::vector<openarm::damiao_motor::MITParam> hand_cmds;
        hand_cmds.reserve(hand_motor_refs.size());
        for (size_t i = 0; i < hand_motor_refs.size(); ++i) {
            hand_cmds.emplace_back(openarm::damiao_motor::MITParam{
                Kp_[i + arm_dof], Kd_[i + arm_dof], hand_motor_refs[i].position,
                hand_motor_refs[i].velocity, 0.0});
        }

        openarm_->get_arm().mit_control_all(arm_cmds);
        openarm_->get_gripper().mit_control_all(hand_cmds);

        openarm_->recv_all(200);

        return true;
    }

    return true;
}

void Control::ComputeFriction(const double* velocity, double* friction, size_t index) {
    if (TANHFRIC) {
        const double amp_tmp = 1.0;
        const double coef_tmp = 0.1;

        const double v = velocity[index];
        const double Fc = Fc_.at(index);
        const double k = k_.at(index);
        const double Fv = Fv_.at(index);
        const double Fo = Fo_.at(index);

        friction[index] = amp_tmp * Fc * std::tanh(coef_tmp * k * v) + Fv * v + Fo;
    } else {
        friction[index] = velocity[index] * Dn_.at(index);
    }
}

bool Control::AdjustPosition(void) {
    int nstep = 220;
    double alpha;

    std::vector<MotorState> arm_motor_states;
    for (const auto& motor : openarm_->get_arm().get_motors()) {
        arm_motor_states.push_back({motor.get_position(), motor.get_velocity(), 0.0});
    }

    std::vector<MotorState> gripper_motor_states;
    for (const auto& motor : openarm_->get_gripper().get_motors()) {
        gripper_motor_states.push_back({motor.get_position(), motor.get_velocity(), 0.0});
    }

    std::vector<JointState> joint_arm_now =
        openarmjointconverter_->motor_to_joint(arm_motor_states);
    std::vector<JointState> joint_hand_now =
        openarmgripperjointconverter_->motor_to_joint(gripper_motor_states);

    std::vector<JointState> joint_arm_goal(NMOTORS - 1);
    for (size_t i = 0; i < NMOTORS - 1; ++i) {
        joint_arm_goal[i].position = INITIAL_POSITION[i];
        joint_arm_goal[i].velocity = 0.0;
        joint_arm_goal[i].effort = 0.0;
    }

    std::vector<JointState> joint_hand_goal(joint_hand_now.size());
    for (size_t i = 0; i < joint_hand_goal.size(); ++i) {
        joint_hand_goal[i].position = 0.0;
        joint_hand_goal[i].velocity = 0.0;
        joint_hand_goal[i].effort = 0.0;
    }

    std::vector<double> kp_arm_temp = {50, 50.0, 50.0, 50.0, 10.0, 10.0, 10.0};
    std::vector<double> kd_arm_temp = {1.2, 1.2, 1.2, 1.2, 0.3, 0.2, 0.3};

    std::vector<double> kp_hand_temp = {10.0};
    std::vector<double> kd_hand_temp = {0.5};

    for (int step = 0; step < nstep; ++step) {
        alpha = static_cast<double>(step + 1) / nstep;

        std::vector<JointState> joint_arm_interp(NMOTORS - 1);
        for (size_t i = 0; i < NMOTORS - 1; ++i) {
            joint_arm_interp[i].position =
                joint_arm_goal[i].position * alpha + joint_arm_now[i].position * (1.0 - alpha);
            joint_arm_interp[i].velocity = 0.0;
        }

        std::vector<JointState> joint_hand_interp(joint_hand_goal.size());
        for (size_t i = 0; i < joint_hand_interp.size(); ++i) {
            joint_hand_interp[i].position =
                joint_hand_goal[i].position * alpha + joint_hand_now[i].position * (1.0 - alpha);
            joint_hand_interp[i].velocity = 0.0;
        }

        std::vector<MotorState> arm_motor_refs =
            openarmjointconverter_->joint_to_motor(joint_arm_interp);
        std::vector<MotorState> hand_motor_refs =
            openarmgripperjointconverter_->joint_to_motor(joint_hand_interp);

        std::vector<openarm::damiao_motor::MITParam> arm_cmds;
        arm_cmds.reserve(arm_motor_refs.size());
        for (size_t i = 0; i < arm_motor_refs.size(); ++i) {
            arm_cmds.emplace_back(openarm::damiao_motor::MITParam{kp_arm_temp[i], kd_arm_temp[i],
                                                                  arm_motor_refs[i].position,
                                                                  arm_motor_refs[i].velocity, 0.0});
        }

        std::vector<openarm::damiao_motor::MITParam> hand_cmds;
        hand_cmds.reserve(hand_motor_refs.size());
        for (size_t i = 0; i < hand_motor_refs.size(); ++i) {
            hand_cmds.emplace_back(openarm::damiao_motor::MITParam{
                kp_hand_temp[i], kd_hand_temp[i], hand_motor_refs[i].position,
                hand_motor_refs[i].velocity, 0.0});
        }

        openarm_->get_arm().mit_control_all(arm_cmds);
        openarm_->get_gripper().mit_control_all(hand_cmds);

        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        openarm_->recv_all();
    }

    std::vector<MotorState> arm_motor_states_final;
    for (const auto& motor : openarm_->get_arm().get_motors()) {
        arm_motor_states_final.push_back({motor.get_position(), motor.get_velocity(), 0.0});
    }

    std::vector<MotorState> gripper_motor_states_final;
    for (const auto& motor : openarm_->get_gripper().get_motors()) {
        gripper_motor_states_final.push_back({motor.get_position(), motor.get_velocity(), 0.0});
    }

    std::vector<JointState> joint_arm_final =
        openarmjointconverter_->motor_to_joint(arm_motor_states_final);
    std::vector<JointState> joint_hand_final =
        openarmgripperjointconverter_->motor_to_joint(gripper_motor_states_final);

    robot_state_->arm_state().set_all_references(joint_arm_final);
    robot_state_->hand_state().set_all_references(joint_hand_final);

    return true;
}

bool Control::DetectVibration(const double* velocity, bool* what_axis) {
    bool vibration_detected = false;

    for (int i = 0; i < NJOINTS; ++i) {
        what_axis[i] = false;

        velocity_buffer_[i].push_back(velocity[i]);
        if (velocity_buffer_[i].size() > VEL_WINDOW_SIZE) velocity_buffer_[i].pop_front();

        if (velocity_buffer_[i].size() < VEL_WINDOW_SIZE) continue;

        double mean = std::accumulate(velocity_buffer_[i].begin(), velocity_buffer_[i].end(), 0.0) /
                      velocity_buffer_[i].size();

        double var = 0.0;
        for (double v : velocity_buffer_[i]) {
            var += (v - mean) * (v - mean);
        }

        double stddev = std::sqrt(var / velocity_buffer_[i].size());

        if (stddev > VIB_THRESHOLD) {
            what_axis[i] = true;
            vibration_detected = true;
            std::cout << "[VIBRATION] Joint " << i << " stddev: " << stddev << std::endl;
        }
    }

    return vibration_detected;
}