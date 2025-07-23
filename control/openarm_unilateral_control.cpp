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
#include <thread>
#include <csignal>
#include <atomic>
#include <filesystem>

#include <periodic_timer_thread.hpp>
#include <robot_state.hpp>
#include <openarm/can/socket/openarm.hpp>
#include <openarm/damiao_motor/dm_motor_constants.hpp>
#include <dynamics.hpp>
#include <yamlloader.hpp>
#include <control.hpp>

std::atomic<bool> keep_running(true);


void signal_handler(int signal) {
    if (signal == SIGINT) {
        std::cout << "\nCtrl+C detected. Exiting loop..." << std::endl;
        keep_running = false;
    }
}


class LeaderArmThread : public PeriodicTimerThread {
    public:
        LeaderArmThread(std::shared_ptr<RobotSystemState> robot_state, Control *control_l, double hz = 500.0)
            : PeriodicTimerThread(hz), robot_state_(robot_state), control_l_(control_l){}
    
    protected:

        void before_start() override {
            std::cout << "leader start thread " << std::endl;
        }

        void after_stop() override {
            std::cout << "leader stop thread " << std::endl;
        }

        void on_timer() override {
            static auto prev_time = std::chrono::steady_clock::now();

            control_l_->DoControl_u();

            auto now = std::chrono::steady_clock::now();

            auto elapsed_us = std::chrono::duration_cast<std::chrono::microseconds>(now - prev_time).count();
            prev_time = now;
        
            // std::cout << "[Leader] Period: " << elapsed_us << " us" << std::endl;
            }
    
    private:
        std::shared_ptr<RobotSystemState> robot_state_;
        Control *control_l_;

    };


class FollowerArmThread : public PeriodicTimerThread {
    public:
        FollowerArmThread(std::shared_ptr<RobotSystemState> robot_state, Control *control_f, double hz = 500.0)
            : PeriodicTimerThread(hz), robot_state_(robot_state), control_f_(control_f)  {}
    
    protected:
        void before_start() override {
            std::cout << "follower start thread " << std::endl;
        }

        void after_stop() override {
            std::cout << "follower stop thread " << std::endl;
        }

        void on_timer() override {
            static auto prev_time = std::chrono::steady_clock::now();

            control_f_->DoControl_u();
        
            auto now = std::chrono::steady_clock::now();

            auto elapsed_us = std::chrono::duration_cast<std::chrono::microseconds>(now - prev_time).count();
            prev_time = now;
        
            // std::cout << "[Follower] Period: " << elapsed_us << " us" << std::endl;
        }
    
    private:
        std::shared_ptr<RobotSystemState> robot_state_;
        Control *control_f_;

    };


class AdminThread : public PeriodicTimerThread {
    public:
        AdminThread(std::shared_ptr<RobotSystemState> leader_state,
                    std::shared_ptr<RobotSystemState> follower_state,
                    Control *control_l,
                    Control *control_f,
                    double hz = 500.0)
            : PeriodicTimerThread(hz), leader_state_(leader_state), follower_state_(follower_state), control_l_(control_l), control_f_(control_f)  {}
    
    protected:
        void before_start() override {
            std::cout << "admin start thread " << std::endl;
        }

        void after_stop() override {
            std::cout << "admin stop thread " << std::endl;
        }

        void on_timer() override {
            
            static auto prev_time = std::chrono::steady_clock::now();
            auto now = std::chrono::steady_clock::now();
            
            // get response
            auto leader_arm_resp = leader_state_->arm_state().get_all_responses();
            auto follower_arm_resp = follower_state_->arm_state().get_all_responses();
    
            auto leader_hand_resp = leader_state_->hand_state().get_all_responses();
            auto follower_hand_resp = follower_state_->hand_state().get_all_responses();

            // --- print responses ---
            // std::cerr << "[Leader Arm Response]" << std::endl;
            // for (size_t i = 0; i < leader_arm_resp.size(); ++i) {
            //     const auto& j = leader_arm_resp[i];
            //     std::cerr << "  joint[" << i << "] = {"
            //             << j.position << ", "
            //             << j.velocity << ", "
            //             << j.effort << "}" << std::endl;
            // }

            // std::cerr << "[Follower Arm Response]" << std::endl;
            // for (size_t i = 0; i < follower_arm_resp.size(); ++i) {
            //     const auto& j = follower_arm_resp[i];
            //     std::cerr << "  joint[" << i << "] = {"
            //             << j.position << ", "
            //             << j.velocity << ", "
            //             << j.effort << "}" << std::endl;
            // }

            // std::cerr << "[Leader Hand Response]" << std::endl;
            // for (size_t i = 0; i < leader_hand_resp.size(); ++i) {
            //     const auto& j = leader_hand_resp[i];
            //     std::cerr << "  joint[" << i << "] = {"
            //             << j.position << ", "
            //             << j.velocity << ", "
            //             << j.effort << "}" << std::endl;
            // }

            // std::cerr << "[Follower Hand Response]" << std::endl;
            // for (size_t i = 0; i < follower_hand_resp.size(); ++i) {
            //     const auto& j = follower_hand_resp[i];
            //     std::cerr << "  joint[" << i << "] = {"
            //             << j.position << ", "
            //             << j.velocity << ", "
            //             << j.effort << "}" << std::endl;
            // }

            //set referense
            leader_state_->arm_state().set_all_references(follower_arm_resp);
            leader_state_->hand_state().set_all_references(follower_hand_resp);
        
            follower_state_->arm_state().set_all_references(leader_arm_resp);
            follower_state_->hand_state().set_all_references(leader_hand_resp);
        
            auto elapsed_us = std::chrono::duration_cast<std::chrono::microseconds>(now - prev_time).count();
            prev_time = now;
        
            // std::cout << "[Admin] Period: " << elapsed_us << " us" << std::endl; 
            }
    
    private:
        std::shared_ptr<RobotSystemState> leader_state_;
        std::shared_ptr<RobotSystemState> follower_state_;
        Control *control_l_;
        Control *control_f_;

    };


int main(int argc, char** argv) {
    try {
        std::signal(SIGINT, signal_handler);

        std::string arm_side = "right_arm";
        std::string leader_urdf_path;
        std::string follower_urdf_path;
        std::string leader_can_interface = "can0";
        std::string follower_can_interface = "can1";

        if (argc < 3) {
            std::cerr << "Usage: " << argv[0] << " <leader_urdf_path> <follower_urdf_path> [arm_side] [leader_can] [follower_can]" << std::endl;
            return 1;
        }

        // Required: URDF paths
        leader_urdf_path = argv[1];
        follower_urdf_path = argv[2];

        // Optional: arm_side
        if (argc >= 4) {
            arm_side = argv[3];
            if (arm_side != "left_arm" && arm_side != "right_arm") {
                std::cerr << "[ERROR] Invalid arm_side: " << arm_side << ". Must be 'left_arm' or 'right_arm'." << std::endl;
                return 1;
            }
        }

        // Optional: CAN interfaces
        if (argc >= 6) {
            leader_can_interface = argv[4];
            follower_can_interface = argv[5];
        }

        // URDF file existence check
        if (!std::filesystem::exists(leader_urdf_path)) {
            std::cerr << "[ERROR] Leader URDF not found: " << leader_urdf_path << std::endl;
            return 1;
        }
        if (!std::filesystem::exists(follower_urdf_path)) {
            std::cerr << "[ERROR] Follower URDF not found: " << follower_urdf_path << std::endl;
            return 1;
        }

        // Setup dynamics
        std::string root_link = "openarm_body_link0";
        std::string leaf_link = (arm_side == "left_arm") ? "openarm_left_hand" : "openarm_right_hand";

        // Output confirmation
        std::cout << "=== OpenArm Unilateral Control ===" << std::endl;
        std::cout << "Arm side         : " << arm_side << std::endl;
        std::cout << "Leader CAN       : " << leader_can_interface << std::endl;
        std::cout << "Follower CAN     : " << follower_can_interface << std::endl;
        std::cout << "Leader URDF path : " << leader_urdf_path << std::endl;
        std::cout << "Follower URDF path: " << follower_urdf_path << std::endl;
        std::cout << "Root link         : " << root_link << std::endl;
        std::cout << "Leaf link         : " << leaf_link << std::endl;

        YamlLoader leader_loader("config/leader.yaml");
        YamlLoader follower_loader("config/follower.yaml");

        // Leader parameters
        std::vector<double> leader_kp = leader_loader.get_vector("LeaderArmParam", "Kp");
        std::vector<double> leader_kd = leader_loader.get_vector("LeaderArmParam", "Kd");
        std::vector<double> leader_kf = leader_loader.get_vector("LeaderArmParam", "Kf");
        std::vector<double> leader_gn = leader_loader.get_vector("LeaderArmParam", "gn");
        std::vector<double> leader_Dn = leader_loader.get_vector("LeaderArmParam", "Dn");
        std::vector<double> leader_Jn = leader_loader.get_vector("LeaderArmParam", "Jn");
        std::vector<double> leader_Fc = leader_loader.get_vector("LeaderArmParam", "Fc");
        std::vector<double> leader_k  = leader_loader.get_vector("LeaderArmParam", "k");
        std::vector<double> leader_Fv = leader_loader.get_vector("LeaderArmParam", "Fv");
        std::vector<double> leader_Fo = leader_loader.get_vector("LeaderArmParam", "Fo");

        // Follower parameters
        std::vector<double> follower_kp = follower_loader.get_vector("FollowerArmParam", "Kp");
        std::vector<double> follower_kd = follower_loader.get_vector("FollowerArmParam", "Kd");
        std::vector<double> follower_kf = follower_loader.get_vector("FollowerArmParam", "Kf");
        std::vector<double> follower_gn = follower_loader.get_vector("FollowerArmParam", "gn");
        std::vector<double> follower_Dn = follower_loader.get_vector("FollowerArmParam", "Dn");
        std::vector<double> follower_Jn = follower_loader.get_vector("FollowerArmParam", "Jn");
        std::vector<double> follower_Fc = follower_loader.get_vector("FollowerArmParam", "Fc");
        std::vector<double> follower_k  = follower_loader.get_vector("FollowerArmParam", "k");
        std::vector<double> follower_Fv = follower_loader.get_vector("FollowerArmParam", "Fv");
        std::vector<double> follower_Fo = follower_loader.get_vector("FollowerArmParam", "Fo");

        // --- Print utility ---
        // auto print_vector = [](const std::string& name, const std::vector<double>& vec) {
        //     std::cout << name << " = [ ";
        //     for (const auto& v : vec) {
        //         std::cout << v << " ";
        //     }
        //     std::cout << "]\n";
        // };

        // // --- Print all ---
        // print_vector("leader_kp", leader_kp);
        // print_vector("leader_kd", leader_kd);
        // print_vector("leader_kf", leader_kf);
        // print_vector("leader_gn", leader_gn);
        // print_vector("leader_Dn", leader_Dn);
        // print_vector("leader_Fc", leader_Fc);
        // print_vector("leader_k",  leader_k);
        // print_vector("leader_Fv", leader_Fv);
        // print_vector("leader_Fo", leader_Fo);

        // print_vector("follower_kp", follower_kp);
        // print_vector("follower_kd", follower_kd);
        // print_vector("follower_kf", follower_kf);
        // print_vector("follower_gn", follower_gn);
        // print_vector("follower_Dn", follower_Dn);
        // print_vector("follower_Fc", follower_Fc);
        // print_vector("follower_k",  follower_k);
        // print_vector("follower_Fv", follower_Fv);
        // print_vector("follower_Fo", follower_Fo);

        Dynamics *leader_arm_dynamics = new Dynamics(leader_urdf_path, root_link, leaf_link);
        leader_arm_dynamics->Init();

        Dynamics *follower_arm_dynamics = new Dynamics(follower_urdf_path, root_link, leaf_link);
        follower_arm_dynamics->Init();

        openarm::can::socket::OpenArm* leader_openarm = new openarm::can::socket::OpenArm(leader_can_interface, true);
        openarm::can::socket::OpenArm* follower_openarm = new openarm::can::socket::OpenArm(follower_can_interface, true);
        

        // Initialize arm motors
        std::vector<openarm::damiao_motor::MotorType> leader_motor_types = {
            openarm::damiao_motor::MotorType::DM8009, openarm::damiao_motor::MotorType::DM8009,
            openarm::damiao_motor::MotorType::DM4340, openarm::damiao_motor::MotorType::DM4340,
            openarm::damiao_motor::MotorType::DM4310, openarm::damiao_motor::MotorType::DM4310,
            openarm::damiao_motor::MotorType::DM4310
        };

        // Initialize arm motors
        std::vector<openarm::damiao_motor::MotorType> follower_motor_types = {
            openarm::damiao_motor::MotorType::DM8009, openarm::damiao_motor::MotorType::DM8009,
            openarm::damiao_motor::MotorType::DM4340, openarm::damiao_motor::MotorType::DM4340,
            openarm::damiao_motor::MotorType::DM4310, openarm::damiao_motor::MotorType::DM4310,
            openarm::damiao_motor::MotorType::DM4310
        };

        std::vector<uint32_t> leader_send_can_ids = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07};
        std::vector<uint32_t> leader_recv_can_ids = {0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17};
        std::cout << "Initializing leader arm..." << std::endl;
        leader_openarm->init_arm_motors(leader_motor_types, leader_send_can_ids, leader_recv_can_ids);

        std::vector<uint32_t> follower_send_can_ids = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07};
        std::vector<uint32_t> follower_recv_can_ids = {0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17};
        std::cout << "Initializing follower arm..." << std::endl;
        follower_openarm->init_arm_motors(follower_motor_types, follower_send_can_ids, follower_recv_can_ids);

        // Initialize gripper
        std::cout << "Initializing leader gripper..." << std::endl;
        leader_openarm->init_gripper_motor(openarm::damiao_motor::MotorType::DM4310, 0x08, 0x18);
        leader_openarm->set_callback_mode_all(openarm::damiao_motor::CallbackMode::STATE);

        // Initialize gripper
        std::cout << "Initializing follower gripper..." << std::endl;
        follower_openarm->init_gripper_motor(openarm::damiao_motor::MotorType::DM4310, 0x08, 0x18);
        follower_openarm->set_callback_mode_all(openarm::damiao_motor::CallbackMode::STATE);

        // Enable all motors
        std::cout << "=== Enabling Leader Motors ===" << std::endl;
        leader_openarm->enable_all();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        leader_openarm->recv_all();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        // Enable all motors
        std::cout << "=== Enabling Follower Motors ===" << std::endl;
        follower_openarm->enable_all();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        follower_openarm->recv_all();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));        

        size_t leader_arm_motor_num = leader_openarm->get_arm().get_motors().size();
        size_t follower_arm_motor_num = follower_openarm->get_arm().get_motors().size();
        size_t leader_hand_motor_num = leader_openarm->get_gripper().get_motors().size();
        size_t follower_hand_motor_num = follower_openarm->get_gripper().get_motors().size();

        std::cout << "leader arm motor num : " << leader_arm_motor_num << std::endl;
        std::cout << "follower arm motor num : " << follower_arm_motor_num << std::endl;
        std::cout << "leader hand motor num : " << leader_hand_motor_num << std::endl;
        std::cout << "follower hand motor num : " << follower_hand_motor_num << std::endl;

        // Declare robot_state (Joint and motor counts are assumed to be equal)
        std::shared_ptr<RobotSystemState> leader_state = 
        std::make_shared<RobotSystemState>(leader_arm_motor_num, leader_hand_motor_num);

        std::shared_ptr<RobotSystemState> follower_state = 
            std::make_shared<RobotSystemState>(follower_arm_motor_num, follower_hand_motor_num);
    

        Control* control_leader = new Control(leader_openarm,leader_arm_dynamics,follower_arm_dynamics, leader_state, 1.0 / FREQUENCY, ROLE_LEADER, arm_side, leader_arm_motor_num, leader_hand_motor_num);
        Control* control_follower = new Control(follower_openarm,leader_arm_dynamics,follower_arm_dynamics, follower_state, 1.0 / FREQUENCY, ROLE_FOLLOWER, arm_side, follower_arm_motor_num, follower_hand_motor_num);

        //set parameter
        control_leader->SetParameter(leader_Dn, leader_Jn, leader_gn,
                                     leader_kp, leader_kd, leader_kf,
                                     leader_Fc, leader_k, leader_Fv, leader_Fo);

        control_follower->SetParameter(follower_Dn, follower_Jn, follower_gn,
                                       follower_kp, follower_kd, follower_kf,
                                       follower_Fc, follower_k, follower_Fv, follower_Fo);

        //set home postion
        std::thread thread_l(&Control::AdjustPosition, control_leader);
        std::thread thread_f(&Control::AdjustPosition, control_follower);
        thread_l.join();
        thread_f.join();

        // Start control process
        LeaderArmThread leader_thread(leader_state ,control_leader, FREQUENCY);
        FollowerArmThread follower_thread(follower_state, control_follower, FREQUENCY);
        AdminThread admin_thread(leader_state, follower_state, control_leader, control_follower, FREQUENCY);

        leader_thread.start_thread();
        follower_thread.start_thread();
        admin_thread.start_thread();
    
        while (keep_running) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        
        leader_thread.stop_thread();
        follower_thread.stop_thread();
        admin_thread.stop_thread();
    
        leader_openarm->disable_all();
        follower_openarm->disable_all();


    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }

    


    return 0;
}