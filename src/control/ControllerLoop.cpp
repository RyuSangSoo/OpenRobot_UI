#include "control/ControllerLoop.hpp"

#include <algorithm>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <system_error>
#include <thread>

namespace {

struct TimingAccumulator {
    size_t count = 0;
    double sum_ms = 0.0;
    double sum_sq_ms = 0.0;
    double min_ms = std::numeric_limits<double>::infinity();
    double max_ms = 0.0;

    void add(const double sample_ms) {
        ++count;
        sum_ms += sample_ms;
        sum_sq_ms += sample_ms * sample_ms;
        min_ms = std::min(min_ms, sample_ms);
        max_ms = std::max(max_ms, sample_ms);
    }

    double avg_ms() const {
        return count ? (sum_ms / static_cast<double>(count)) : 0.0;
    }

    double stddev_ms() const {
        if (count < 2) return 0.0;
        const double mean = avg_ms();
        const double variance = std::max(0.0, (sum_sq_ms / static_cast<double>(count)) - mean * mean);
        return std::sqrt(variance);
    }

    void reset() {
        *this = TimingAccumulator{};
    }
};

void publish_control_timing(const TimingAccumulator& period_stats,
                            const TimingAccumulator& cycle_stats) {
    const double avg_period_ms = period_stats.avg_ms();
    g_control_timing_report.hz.store(avg_period_ms > 0.0 ? (1000.0 / avg_period_ms) : 0.0);
    g_control_timing_report.period_avg_ms.store(avg_period_ms);
    g_control_timing_report.period_min_ms.store(
        std::isfinite(period_stats.min_ms) ? period_stats.min_ms : 0.0);
    g_control_timing_report.period_max_ms.store(period_stats.max_ms);
    g_control_timing_report.period_std_ms.store(period_stats.stddev_ms());
    g_control_timing_report.read_avg_ms.store(0.0);
    g_control_timing_report.read_max_ms.store(0.0);
    g_control_timing_report.write_avg_ms.store(0.0);
    g_control_timing_report.write_max_ms.store(0.0);
    g_control_timing_report.cycle_avg_ms.store(cycle_stats.avg_ms());
    g_control_timing_report.cycle_max_ms.store(cycle_stats.max_ms);
    g_control_timing_report.seq.fetch_add(1, std::memory_order_relaxed);
}

}  // namespace

ControllerLoop::ControllerLoop(const std::string& urdf_path,
                               std::filesystem::path test_log_dir)
    : urdf_path_(urdf_path),
      test_log_dir_(std::move(test_log_dir)) {}

void ControllerLoop::Run() {
    MjuJoy mjujoy("/dev/input/js0");
    MjuRobot robot(urdf_path_);

    auto last_time = std::chrono::steady_clock::now();
    auto next_loop_time = last_time + std::chrono::duration<double>(dt);
    double t_global = 0.0;

    Mode mode = INIT;
    Mode prev_mode = mode;
    g_current_mode.store(static_cast<int>(mode));

    installSignalHandler();
    robot.PatternInit();

    double button_count = 0.0;
    double prev_RB = 0.0;
    double prev_LB = 0.0;
    bool emergency_stop_latched = false;
    bool prev_estop_combo = false;
    std::ofstream test_log_file;
    int test_log_joint_idx = -1;
    double prev_test_log_vel = 0.0;
    bool test_log_vel_initialized = false;
    g_fsm_mode_count.store(0);
    unsigned long long prev_hw_execute_rearm_counter = g_hw_execute_rearm_counter.load();
    auto prev_cycle_start = std::chrono::steady_clock::now();
    auto last_timing_log_time = prev_cycle_start;
    size_t cycle_count = 0;
    TimingAccumulator period_stats;
    TimingAccumulator cycle_stats;

    auto OpenTestLog = [&]() {
        std::error_code ec;
        std::filesystem::create_directories(test_log_dir_, ec);
        if (ec) {
            std::cerr << "Failed to create log directory: " << test_log_dir_
                      << " (" << ec.message() << ")\n";
            return;
        }

        const auto now = std::chrono::system_clock::now();
        const std::time_t tt = std::chrono::system_clock::to_time_t(now);
        std::tm tm_buf{};
        localtime_r(&tt, &tm_buf);

        std::ostringstream oss;
        oss << "test_" << std::put_time(&tm_buf, "%Y%m%d_%H%M%S") << ".csv";
        const std::filesystem::path file_path = test_log_dir_ / oss.str();

        test_log_file.open(file_path, std::ios::out | std::ios::trunc);
        if (!test_log_file.is_open()) {
            std::cerr << "Failed to open test log file: " << file_path << std::endl;
            return;
        }

        test_log_file << "t_global;joint_idx;q;qdot;qdd;tau_id;tau_pd;tau_fric;tau_cmd;ff_enable;amp;freq_hz\n";
        test_log_file.flush();
        std::cout << "TEST log started: " << file_path << std::endl;
    };

    auto CloseTestLog = [&]() {
        if (test_log_file.is_open()) {
            test_log_file.flush();
            test_log_file.close();
            std::cout << "TEST log stopped" << std::endl;
        }
    };

    while (keepRunning) {
        const auto cycle_start = std::chrono::steady_clock::now();
        if (cycle_count > 0) {
            period_stats.add(
                std::chrono::duration<double, std::milli>(cycle_start - prev_cycle_start).count());
        }

        robot.StateUpdate();
        mjujoy.update();

        const bool estop_combo = (mjujoy.JOYBtnRB == 1.0 && mjujoy.JOYBtnLB == 1.0);
        if (estop_combo && !prev_estop_combo) {
            emergency_stop_latched = true;
            mode = INIT;
            std::cout << "E-STOP: MODE -> INIT, torque = 0" << std::endl;

            t_global = 0.0;
            last_time = std::chrono::steady_clock::now();

            mjustate.Sim.x_init = mjustate.Sim.Tip_pos.x();
            mjustate.Sim.y_init = mjustate.Sim.Tip_pos.y();
            mjustate.Sim.z_init = mjustate.Sim.Tip_pos.z();
            mjustate.Sim.Pos.joint1_init = mjustate.Sim.Pos.joint1;
            mjustate.Sim.Pos.joint2_init = mjustate.Sim.Pos.joint2;
            mjustate.Sim.Pos.joint3_init = mjustate.Sim.Pos.joint3;
            mjustate.Sim.Pos.joint4_init = mjustate.Sim.Pos.joint4;
        }
        prev_estop_combo = estop_combo;

        if (mjujoy.JOYBtnRB == 1.0 && prev_RB == 0.0) {
            button_count++;
            std::cout << "count : " << button_count << std::endl;
        }
        if (mjujoy.JOYBtnLB == 1.0 && prev_LB == 0.0) {
            button_count = 0;
        }
        g_fsm_mode_count.store(static_cast<int>(std::clamp(button_count, 0.0, 3.0)));
        prev_RB = mjujoy.JOYBtnRB;
        prev_LB = mjujoy.JOYBtnLB;

        const unsigned long long cur_rearm_counter = g_hw_execute_rearm_counter.load();
        if (cur_rearm_counter != prev_hw_execute_rearm_counter) {
            prev_hw_execute_rearm_counter = cur_rearm_counter;
            t_global = 0.0;
            last_time = std::chrono::steady_clock::now();
            mjustate.Sim.x_init = mjustate.Sim.Tip_pos.x();
            mjustate.Sim.y_init = mjustate.Sim.Tip_pos.y();
            mjustate.Sim.z_init = mjustate.Sim.Tip_pos.z();
            mjustate.Sim.Pos.joint1_init = mjustate.Sim.Pos.joint1;
            mjustate.Sim.Pos.joint2_init = mjustate.Sim.Pos.joint2;
            mjustate.Sim.Pos.joint3_init = mjustate.Sim.Pos.joint3;
            mjustate.Sim.Pos.joint4_init = mjustate.Sim.Pos.joint4;
        }

        if (emergency_stop_latched && mjujoy.JOYBtnY == 1 && mode == INIT) {
            emergency_stop_latched = false;
            std::cout << "E-STOP released" << std::endl;
        }

        if (!emergency_stop_latched && mjujoy.JOYBtnY == 1 && mode == INIT) {
            mode = READY;
            std::cout << "MODE : READY" << std::endl;
            t_global = 0.0;
            last_time = std::chrono::steady_clock::now();
            mjustate.Sim.x_init = mjustate.Sim.Tip_pos.x();
            mjustate.Sim.y_init = mjustate.Sim.Tip_pos.y();
            mjustate.Sim.z_init = mjustate.Sim.Tip_pos.z();
            mjustate.Sim.Pos.joint1_init = mjustate.Sim.Pos.joint1;
            mjustate.Sim.Pos.joint2_init = mjustate.Sim.Pos.joint2;
            mjustate.Sim.Pos.joint3_init = mjustate.Sim.Pos.joint3;
            mjustate.Sim.Pos.joint4_init = mjustate.Sim.Pos.joint4;
        } else if (mjujoy.JOYBtnX == 1 && mode == READY) {
            mode = STAND;
            std::cout << "MODE : STAND" << std::endl;
            t_global = 0.0;
            last_time = std::chrono::steady_clock::now();
            mjustate.Sim.x_init = mjustate.Sim.Tip_pos.x();
            mjustate.Sim.y_init = mjustate.Sim.Tip_pos.y();
            mjustate.Sim.z_init = mjustate.Sim.Tip_pos.z();

            mjustate.Ref.x_init = mjustate.Ref.Tip_pos.x();
            mjustate.Ref.y_init = mjustate.Ref.Tip_pos.y();
            mjustate.Ref.z_init = mjustate.Ref.Tip_pos.z();
        } else if (mjujoy.JOYBtnB == 1 && mode == INIT) {
            mode = TEST;
            std::cout << "MODE : TEST" << std::endl;
            t_global = 0.0;
            last_time = std::chrono::steady_clock::now();
            mjustate.Sim.Pos.joint1_init = mjustate.Sim.Pos.joint1;
            mjustate.Sim.Pos.joint2_init = mjustate.Sim.Pos.joint2;
            mjustate.Sim.Pos.joint3_init = mjustate.Sim.Pos.joint3;
            mjustate.Sim.Pos.joint4_init = mjustate.Sim.Pos.joint4;
        } else if (mjujoy.JOYBtnA == 1 && mode == STAND) {
            mode = WALKING;
            std::cout << "MODE : WALKING" << std::endl;
            t_global = 0.0;
            last_time = std::chrono::steady_clock::now();
            mjustate.Sim.x_init = mjustate.Sim.Tip_pos.x();
            mjustate.Sim.y_init = mjustate.Sim.Tip_pos.y();
            mjustate.Sim.z_init = mjustate.Sim.Tip_pos.z();
        } else if (mjujoy.JOYBtnB == 1 && mode == STAND) {
            mode = SWIMMING;
            std::cout << "MODE : SWIMMING" << std::endl;
            t_global = 0.0;
            last_time = std::chrono::steady_clock::now();
            mjustate.Sim.x_init = mjustate.Sim.Tip_pos.x();
            mjustate.Sim.y_init = mjustate.Sim.Tip_pos.y();
            mjustate.Sim.z_init = mjustate.Sim.Tip_pos.z();
            mjustate.Sim.Pos.joint1_init = mjustate.Sim.Pos.joint1;
            mjustate.Sim.Pos.joint2_init = mjustate.Sim.Pos.joint2;
            mjustate.Sim.Pos.joint3_init = mjustate.Sim.Pos.joint3;
            mjustate.Sim.Pos.joint4_init = mjustate.Sim.Pos.joint4;
        } else if (mjujoy.JOYBtnX == 1 && (mode == WALKING || mode == SWIMMING)) {
            mode = STAND;
            std::cout << "MODE : STAND" << std::endl;
            t_global = 0.0;
            last_time = std::chrono::steady_clock::now();
            mjustate.Sim.x_init = mjustate.Sim.Tip_pos.x();
            mjustate.Sim.y_init = mjustate.Sim.Tip_pos.y();
            mjustate.Sim.z_init = mjustate.Sim.Tip_pos.z();
        } else if (mjujoy.JOYBtnX == 1 && mode == TEST) {
            mode = INIT;
            std::cout << "MODE : INIT" << std::endl;
            t_global = 0.0;
            last_time = std::chrono::steady_clock::now();
        } else if (mjujoy.JOYBtnY == 1 && mode == STAND) {
            mode = READY;
            std::cout << "MODE : READY" << std::endl;
            t_global = 0.0;
            last_time = std::chrono::steady_clock::now();
            mjustate.Sim.x_init = mjustate.Sim.Tip_pos.x();
            mjustate.Sim.y_init = mjustate.Sim.Tip_pos.y();
            mjustate.Sim.z_init = mjustate.Sim.Tip_pos.z();
            mjustate.Sim.Pos.joint1_init = mjustate.Sim.Pos.joint1;
            mjustate.Sim.Pos.joint2_init = mjustate.Sim.Pos.joint2;
            mjustate.Sim.Pos.joint3_init = mjustate.Sim.Pos.joint3;
            mjustate.Sim.Pos.joint4_init = mjustate.Sim.Pos.joint4;
        }
        g_current_mode.store(static_cast<int>(mode));

        if (mode == TEST && prev_mode != TEST) {
            robot.ResetTestModeState();
            test_log_joint_idx = -1;
            test_log_vel_initialized = false;
            prev_test_log_vel = 0.0;
            OpenTestLog();
        } else if (mode != TEST && prev_mode == TEST) {
            robot.ResetTestModeState();
            CloseTestLog();
        }

        switch (mode) {
            case INIT: {
                auto current_time_ready = std::chrono::steady_clock::now();
                double elapsed_ready = std::chrono::duration<double>(current_time_ready - last_time).count();
                t_global += elapsed_ready;
                last_time = current_time_ready;
                robot.InitMode(t_global, dt);
                break;
            }
            case READY: {
                auto current_time_ready = std::chrono::steady_clock::now();
                double elapsed_ready = std::chrono::duration<double>(current_time_ready - last_time).count();
                t_global += elapsed_ready;
                last_time = current_time_ready;
                robot.ReadyMode(t_global, dt);
                break;
            }
            case TEST: {
                auto current_time_test = std::chrono::steady_clock::now();
                double elapsed_test = std::chrono::duration<double>(current_time_test - last_time).count();
                t_global += elapsed_test;
                last_time = current_time_test;
                robot.TestMode(t_global, dt);
                break;
            }
            case STAND: {
                auto current_time_stand = std::chrono::steady_clock::now();
                double elapsed_stand = std::chrono::duration<double>(current_time_stand - last_time).count();
                t_global += elapsed_stand;
                last_time = current_time_stand;
                robot.StandMode(t_global, dt);
                break;
            }
            case WALKING: {
                auto current_time_walking = std::chrono::steady_clock::now();
                double elapsed_walking = std::chrono::duration<double>(current_time_walking - last_time).count();
                t_global += elapsed_walking;
                last_time = current_time_walking;
                robot.WalkingMode(button_count, mjujoy.JOYdx, mjujoy.JOYdyaw, mjujoy.JOYroll, 0, 0);
                break;
            }
            case SWIMMING: {
                auto current_time_swimming = std::chrono::steady_clock::now();
                double elapsed_swimming = std::chrono::duration<double>(current_time_swimming - last_time).count();
                t_global += elapsed_swimming;
                last_time = current_time_swimming;
                robot.SwimmingMode(t_global, dt);
                break;
            }
            default:
                break;
        }
        if (emergency_stop_latched) {
            mjustate.Ref.Tau.joint1 = 0.0;
            mjustate.Ref.Tau.joint2 = 0.0;
            mjustate.Ref.Tau.joint3 = 0.0;
            mjustate.Ref.Tau.joint4 = 0.0;
            mjustate.Sim.Tau.joint1 = 0.0;
            mjustate.Sim.Tau.joint2 = 0.0;
            mjustate.Sim.Tau.joint3 = 0.0;
            mjustate.Sim.Tau.joint4 = 0.0;
        } else {
            robot.UpdateTorqueCommand(dt);
        }

        robot.diffTrans();

        if (mode == TEST && test_log_file.is_open()) {
            const int motor_idx = std::max(0, std::min(4, g_test_active_motor_index.load()));
            auto select_motor_value = [&](double m1, double m2, double m3, double m4) {
                if (motor_idx == 4) {
                    return 0.25 * (m1 + m2 + m3 + m4);
                }
                switch (motor_idx) {
                    case 0: return m1;
                    case 1: return m2;
                    case 2: return m3;
                    default: return m4;
                }
            };

            const double r1 = std::max(1e-9, g_motor_gear_ratio[0].load());
            const double r2 = std::max(1e-9, g_motor_gear_ratio[1].load());
            const double r3 = std::max(1e-9, g_motor_gear_ratio[2].load());
            const double r4 = std::max(1e-9, g_motor_gear_ratio[3].load());

            const std::array<double, 4> tau_pd_motor = {
                0.5 * ((g_joint_tau_pd[0].load() / r1) + (g_joint_tau_pd[1].load() / r1)),
                -0.5 * ((g_joint_tau_pd[0].load() / r2) - (g_joint_tau_pd[1].load() / r2)),
                g_joint_tau_pd[2].load() / r3,
                g_joint_tau_pd[3].load() / r4,
            };
            const std::array<double, 4> tau_fric_motor = {
                0.5 * ((g_joint_tau_fric[0].load() / r1) + (g_joint_tau_fric[1].load() / r1)),
                -0.5 * ((g_joint_tau_fric[0].load() / r2) - (g_joint_tau_fric[1].load() / r2)),
                g_joint_tau_fric[2].load() / r3,
                g_joint_tau_fric[3].load() / r4,
            };

            const double q = select_motor_value(
                mjustate.Sim.Pos.motor1, mjustate.Sim.Pos.motor2,
                mjustate.Sim.Pos.motor3, mjustate.Sim.Pos.motor4);
            const double qdot = select_motor_value(
                mjustate.Sim.Vel.motor1, mjustate.Sim.Vel.motor2,
                mjustate.Sim.Vel.motor3, mjustate.Sim.Vel.motor4);
            const double tau_id = select_motor_value(
                mjustate.Ref.Tau.motor1, mjustate.Ref.Tau.motor2,
                mjustate.Ref.Tau.motor3, mjustate.Ref.Tau.motor4);
            const double tau_cmd = select_motor_value(
                mjustate.Sim.Tau.motor1, mjustate.Sim.Tau.motor2,
                mjustate.Sim.Tau.motor3, mjustate.Sim.Tau.motor4);
            const double tau_pd = select_motor_value(
                tau_pd_motor[0], tau_pd_motor[1], tau_pd_motor[2], tau_pd_motor[3]);
            const double tau_fric = select_motor_value(
                tau_fric_motor[0], tau_fric_motor[1], tau_fric_motor[2], tau_fric_motor[3]);

            if (motor_idx != test_log_joint_idx) {
                test_log_joint_idx = motor_idx;
                test_log_vel_initialized = false;
                prev_test_log_vel = qdot;
            }
            const double qdd = test_log_vel_initialized ? (qdot - prev_test_log_vel) / dt : 0.0;
            test_log_vel_initialized = true;
            prev_test_log_vel = qdot;

            test_log_file << t_global << ';'
                          << motor_idx << ';'
                          << q << ';'
                          << qdot << ';'
                          << qdd << ';'
                          << tau_id << ';'
                          << tau_pd << ';'
                          << tau_fric << ';'
                          << tau_cmd << ';'
                          << (g_friction_ff_enable.load() ? 1 : 0) << ';'
                          << g_test_amp.load() << ';'
                          << g_test_freq_hz.load()
                          << '\n';
        }

        prev_mode = mode;
        const auto cycle_end = std::chrono::steady_clock::now();
        cycle_stats.add(std::chrono::duration<double, std::milli>(cycle_end - cycle_start).count());

        const auto timing_log_ms = g_timing_log_ms.load(std::memory_order_relaxed);
        if (timing_log_ms > 0 &&
            std::chrono::duration_cast<std::chrono::milliseconds>(cycle_end - last_timing_log_time)
                    .count() >= timing_log_ms) {
            publish_control_timing(period_stats, cycle_stats);
            period_stats.reset();
            cycle_stats.reset();
            last_timing_log_time = cycle_end;
        }

        prev_cycle_start = cycle_start;
        ++cycle_count;
        next_loop_time += std::chrono::duration<double>(dt);
        std::this_thread::sleep_until(next_loop_time);
    }

    CloseTestLog();
}
