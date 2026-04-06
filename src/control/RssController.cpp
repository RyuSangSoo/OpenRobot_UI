#include "RssController.hpp"

double dt = 0.002;
double joint1_offset = -1.570;
double joint2_offset = 0;
double joint3_offset = 0;
double joint4_offset = 0;
std::array<std::atomic<double>, 4> g_joint_kp = {
    100.0, 100.0, 100.0, 100.0};
std::array<std::atomic<double>, 4> g_joint_kd = {
    8.0, 8.0, 8.0, 8.0};
std::array<std::atomic<double>, 4> g_joint_fv = {
    0.437191, 0.8, 0.6, 1.0};   // viscous coefficient
std::array<std::atomic<double>, 4> g_joint_fc = {
    1.537341, 1.2, 1.0, 1.5};   // Coulomb friction
std::array<std::atomic<double>, 4> g_joint_fs = {
    1.8, 1.8, 1.4, 2.2};   // static friction (Stribeck peak)
std::array<std::atomic<double>, 4> g_joint_vs = {
    0.25, 0.25, 0.2, 0.3}; // Stribeck velocity scale [rad/s]
std::array<std::atomic<double>, 4> g_joint_alpha = {
    1.5, 1.5, 1.5, 1.5};   // Stribeck exponent
std::array<std::atomic<double>, 4> g_joint_eps = {
    0.05, 0.05, 0.05, 0.05}; // tanh smoothing velocity [rad/s]
std::atomic<bool> g_friction_ff_enable{true};
std::array<std::atomic<double>, 4> g_joint_tau_pd = {
    0.0, 0.0, 0.0, 0.0};
std::array<std::atomic<double>, 4> g_joint_tau_fric = {
    0.0, 0.0, 0.0, 0.0};
std::array<std::atomic<double>, 4> g_joint_tau_mj = {
    0.0, 0.0, 0.0, 0.0};
std::atomic<int> g_test_joint_index{0};
std::atomic<int> g_test_active_motor_index{0};
std::atomic<double> g_test_amp{0.25};
std::atomic<double> g_test_freq_hz{0.2};
std::atomic<unsigned long long> g_test_joint_change_counter{0};
std::atomic<int> g_current_mode{0};
std::atomic<int> g_fsm_mode_count{0};
std::atomic<bool> g_hw_execute_enabled{true};
std::atomic<unsigned long long> g_hw_execute_rearm_counter{0};
std::array<std::atomic<double>, 4> g_motor_gear_ratio = {27.0, 27.0, 42.0, 27.0};
std::atomic<uint32_t> g_timing_log_ms{0};
ThreadTimingReport g_ads_rx_timing_report;
ThreadTimingReport g_ads_tx_timing_report;
ThreadTimingReport g_control_timing_report;

PDController::PDController(double kp, double kd)
    : Kp(kp), Kd(kd), previous_error(0.0) {}

void PDController::setGains(double kp, double kd) {
    Kp = kp;
    Kd = kd;
}

double PDController::pdcalculate(double th_ref, double dth_ref, double th, double dth) {
    const double th_error = th_ref - th;
    const double dth_error = dth_ref - dth;
    previous_error = th_error;
    return Kp * th_error + Kd * dth_error;
}

FrictionCompensator::FrictionCompensator(double fv, double fc, double fs,
                                         double vs, double alpha, double eps)
    : Fv(fv), Fc(fc), Fs(fs), Vs(vs), Alpha(alpha), Eps(eps) {}

void FrictionCompensator::setParams(double fv, double fc, double fs,
                                    double vs, double alpha, double eps) {
    Fv = fv;
    Fc = fc;
    Fs = fs;
    Vs = vs;
    Alpha = alpha;
    Eps = eps;
}

double FrictionCompensator::calculate(double dth) const {
    const double vabs = std::abs(dth);
    const double vs_safe = std::max(1e-6, Vs);
    const double eps_safe = std::max(1e-6, Eps);
    const double alpha_safe = std::max(0.5, Alpha);
    const double stribeck =
        Fc + (Fs - Fc) * std::exp(-std::pow(vabs / vs_safe, alpha_safe));
    return Fv * dth + stribeck * std::tanh(dth / eps_safe);
}

// Low Pass Filter Class
LPF::LPF(double tau, double ts)
    : tau_(tau), ts_(ts), prev_output_(0.0) {
        a_ = ts_ / (tau_ + ts_);
        b_ = tau_ / (tau_ + ts_);
    }

double LPF::filter(double input) {
    double output = a_ * input + b_ * prev_output_;
    prev_output_ = output;
    return output;
}


LPF lpf_FL_HY_vel_ready(0.05, dt);
LPF lpf_FL_HP_vel_ready(0.05, dt);
LPF lpf_FL_HR_vel_ready(0.05, dt);
LPF lpf_FL_KN_vel_ready(0.05, dt);
