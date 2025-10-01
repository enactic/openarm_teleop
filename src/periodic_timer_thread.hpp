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
#include <pthread.h>
#include <time.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <iostream>
#include <stdexcept>
#include <thread>

class PeriodicTimerThread {
public:
    explicit PeriodicTimerThread(double hz = 1000.0) : is_running_(false) {
        if (hz <= 0.0) {
            throw std::invalid_argument("Hz must be positive");
        }
        period_us_.store(static_cast<int>(1e6 / hz));
    }

    virtual ~PeriodicTimerThread() { stop_thread(); }

    virtual void start_thread() { start_thread_base(); }

    virtual void stop_thread() { stop_thread_base(); }

    int64_t get_elapsed_time_us() const { return last_elapsed_us_.load(); }

    void set_period(double hz) {
        if (hz > 0.0) {
            period_us_.store(static_cast<int>(1e6 / hz));
        }
    }

protected:
    virtual void on_timer() = 0;

    virtual void before_start() { set_thread_priority(50); }

    virtual void after_stop() {}

    void set_thread_priority(int priority) {
        struct sched_param param;
        param.sched_priority = priority;

        int policy = SCHED_FIFO;

        int result = pthread_setschedparam(pthread_self(), policy, &param);
        if (result != 0) {
            std::cerr << "[WARN] Failed to set real-time priority (errno: " << result
                      << "). Try running with sudo or setcap." << std::endl;
        } else {
            std::cout << "[INFO] Real-time priority set to " << priority << std::endl;
        }
    }

private:
    void start_thread_base() {
        before_start();
        is_running_ = true;
        pthread_create(&thread_, nullptr, &PeriodicTimerThread::thread_entry, this);
    }

    void stop_thread_base() {
        is_running_ = false;
        if (thread_) {
            pthread_join(thread_, nullptr);
            thread_ = 0;
        }
        after_stop();
    }

    static void* thread_entry(void* arg) {
        static_cast<PeriodicTimerThread*>(arg)->timer_thread();
        return nullptr;
    }

    void timer_thread() {
        struct timespec next_time;
        clock_gettime(CLOCK_MONOTONIC, &next_time);

        while (is_running_) {
            auto start = std::chrono::steady_clock::now();

            try {
                on_timer();
            } catch (const std::exception& e) {
                std::cerr << "[ERROR] Exception in on_timer(): " << e.what() << std::endl;
            }

            auto end = std::chrono::steady_clock::now();
            last_elapsed_us_.store(
                std::chrono::duration_cast<std::chrono::microseconds>(end - start).count());

            int period_us = period_us_.load();
            next_time.tv_nsec += period_us * 1000;
            while (next_time.tv_nsec >= 1000000000) {
                next_time.tv_nsec -= 1000000000;
                next_time.tv_sec += 1;
            }
            clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_time, nullptr);
        }
    }

    pthread_t thread_{};
    std::atomic<bool> is_running_;
    std::atomic<int> period_us_;
    std::atomic<int64_t> last_elapsed_us_{0};
};
