#ifndef RSSCONTROLLER_HPP
#define RSSCONTROLLER_HPP

#include "header.hpp"
#include <array>
#include <atomic>

struct ThreadTimingReport {
    std::atomic<unsigned long long> seq {0};
    std::atomic<double> hz {0.0};
    std::atomic<double> period_avg_ms {0.0};
    std::atomic<double> period_min_ms {0.0};
    std::atomic<double> period_max_ms {0.0};
    std::atomic<double> period_std_ms {0.0};
    std::atomic<double> read_avg_ms {0.0};
    std::atomic<double> read_max_ms {0.0};
    std::atomic<double> write_avg_ms {0.0};
    std::atomic<double> write_max_ms {0.0};
    std::atomic<double> cycle_avg_ms {0.0};
    std::atomic<double> cycle_max_ms {0.0};
};

extern double dt;
extern double joint1_offset;
extern double joint2_offset;
extern double joint3_offset;
extern double joint4_offset;
extern std::array<std::atomic<double>, 4> g_joint_kp;
extern std::array<std::atomic<double>, 4> g_joint_kd;
extern std::array<std::atomic<double>, 4> g_joint_fv;
extern std::array<std::atomic<double>, 4> g_joint_fc;
extern std::array<std::atomic<double>, 4> g_joint_fs;
extern std::array<std::atomic<double>, 4> g_joint_vs;
extern std::array<std::atomic<double>, 4> g_joint_alpha;
extern std::array<std::atomic<double>, 4> g_joint_eps;
extern std::atomic<bool> g_friction_ff_enable;
extern std::array<std::atomic<double>, 4> g_joint_tau_pd;
extern std::array<std::atomic<double>, 4> g_joint_tau_fric;
extern std::array<std::atomic<double>, 4> g_joint_tau_mj;
extern std::atomic<int> g_test_joint_index;              // 0..3: motor1..motor4, 4: all
extern std::atomic<int> g_test_active_motor_index;       // currently applied TEST motor target
extern std::atomic<double> g_test_amp;                   // rad
extern std::atomic<double> g_test_freq_hz;               // Hz
extern std::atomic<unsigned long long> g_test_joint_change_counter;  // TEST config changed -> reset sine phase/init
extern std::atomic<int> g_current_mode;                  // Mode enum as int
extern std::atomic<int> g_fsm_mode_count;                // WalkingPattern Mode_count (0..3)
extern std::atomic<bool> g_hw_execute_enabled;           // false: torque hold, true: execute
extern std::atomic<unsigned long long> g_hw_execute_rearm_counter;
extern std::array<std::atomic<double>, 4> g_motor_gear_ratio;
extern std::atomic<uint32_t> g_timing_log_ms;
extern ThreadTimingReport g_ads_rx_timing_report;
extern ThreadTimingReport g_ads_tx_timing_report;
extern ThreadTimingReport g_control_timing_report;

class PDController {
private:
    double Kp;
    double Kd;
    double previous_error;

public:
    PDController(double kp = 0.0, double kd = 0.0);
    void setGains(double kp, double kd);
    double pdcalculate(double th_ref, double dth_ref, double th, double dth);
};

class FrictionCompensator {
private:
    double Fv;
    double Fc;
    double Fs;
    double Vs;
    double Alpha;
    double Eps;

public:
    FrictionCompensator(double fv = 0.0, double fc = 0.0, double fs = 0.0,
                        double vs = 0.1, double alpha = 1.5, double eps = 0.05);
    void setParams(double fv, double fc, double fs, double vs, double alpha, double eps);
    double calculate(double dth) const;
};

// Low Pass Filter
class LPF
{
private:
    double tau_, ts_;
    double a_, b_;
    double prev_output_;

public:
    LPF(double tau, double ts);
    double filter(double input);
};

extern LPF lpf_FL_HY_vel_ready;
extern LPF lpf_FL_HP_vel_ready;
extern LPF lpf_FL_HR_vel_ready;
extern LPF lpf_FL_KN_vel_ready;

#endif
