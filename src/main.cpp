#include "ads/ads_motor_io.hpp"
#include "control/ControllerLoop.hpp"
#include "control/MjuJoint.hpp"
#ifdef ADS_CONTROL_TEST_WITH_ROS2
#include "ros/MjuStatePublisher.hpp"
#endif

#include <algorithm>
#include <array>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <ctime>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace {

constexpr size_t kMotorCount = 4;

struct MotorSpec {
    double gear_ratio = 1.0;
    double kt_nm_per_a = 0.1;
    double counts_per_amp = 1.0;
    double efficiency = 1.0;
};

struct RuntimeOptions {
    std::string urdf_path_arg;
    std::filesystem::path test_log_dir = "logs/test";
    bool debug_refs = false;
    bool read_only = false;
    bool write_only = false;
    uint32_t timing_log_ms = 0;
#ifdef ADS_CONTROL_TEST_WITH_ROS2
    bool ros_publish_enabled = true;
    RosPublisherOptions ros;
#endif
    ads_io::Config ads;
};

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

struct TimingSnapshot {
    unsigned long long seq = 0;
    double hz = 0.0;
    double period_avg_ms = 0.0;
    double period_min_ms = 0.0;
    double period_max_ms = 0.0;
    double period_std_ms = 0.0;
    double read_avg_ms = 0.0;
    double read_max_ms = 0.0;
    double write_avg_ms = 0.0;
    double write_max_ms = 0.0;
    double cycle_avg_ms = 0.0;
    double cycle_max_ms = 0.0;
};

TimingSnapshot load_timing_snapshot(const ThreadTimingReport& report) {
    TimingSnapshot snapshot;
    snapshot.seq = report.seq.load(std::memory_order_relaxed);
    snapshot.hz = report.hz.load(std::memory_order_relaxed);
    snapshot.period_avg_ms = report.period_avg_ms.load(std::memory_order_relaxed);
    snapshot.period_min_ms = report.period_min_ms.load(std::memory_order_relaxed);
    snapshot.period_max_ms = report.period_max_ms.load(std::memory_order_relaxed);
    snapshot.period_std_ms = report.period_std_ms.load(std::memory_order_relaxed);
    snapshot.read_avg_ms = report.read_avg_ms.load(std::memory_order_relaxed);
    snapshot.read_max_ms = report.read_max_ms.load(std::memory_order_relaxed);
    snapshot.write_avg_ms = report.write_avg_ms.load(std::memory_order_relaxed);
    snapshot.write_max_ms = report.write_max_ms.load(std::memory_order_relaxed);
    snapshot.cycle_avg_ms = report.cycle_avg_ms.load(std::memory_order_relaxed);
    snapshot.cycle_max_ms = report.cycle_max_ms.load(std::memory_order_relaxed);
    return snapshot;
}

std::string mode_to_string(const int mode) {
    switch (mode) {
        case INIT: return "INIT";
        case READY: return "READY";
        case TEST: return "TEST";
        case STAND: return "STAND";
        case WALKING: return "WALKING";
        case SWIMMING_READY: return "SWIMMING_READY";
        case SWIMMING: return "SWIMMING";
        default: return "UNKNOWN";
    }
}

std::string format_local_time(const std::chrono::system_clock::time_point tp,
                              const char* format) {
    const std::time_t time = std::chrono::system_clock::to_time_t(tp);
    std::tm tm {};
    localtime_r(&time, &tm);

    std::ostringstream out;
    out << std::put_time(&tm, format);
    return out.str();
}

std::filesystem::path make_timing_csv_path() {
    const std::filesystem::path base_dir =
        std::filesystem::path(__FILE__).parent_path().parent_path() / "test";
    std::error_code ec;
    std::filesystem::create_directories(base_dir, ec);
    if (ec) {
        throw std::runtime_error("Failed to create timing log directory: " + base_dir.string());
    }

    const auto now = std::chrono::system_clock::now();
    return base_dir / (format_local_time(now, "%Y%m%d_%H%M%S") + ".csv");
}

class TimingCsvLogger {
public:
    TimingCsvLogger() : path_(make_timing_csv_path()), out_(path_) {
        if (!out_) {
            throw std::runtime_error("Failed to open timing CSV: " + path_.string());
        }

        out_ << "wall_time,mode,mode_id,fsm_mode_count,"
             << "ads_rx_seq,ads_rx_hz,ads_rx_period_avg_ms,ads_rx_period_min_ms,ads_rx_period_max_ms,ads_rx_period_std_ms,"
             << "ads_rx_read_avg_ms,ads_rx_read_max_ms,ads_rx_cycle_avg_ms,ads_rx_cycle_max_ms,"
             << "ads_tx_seq,ads_tx_hz,ads_tx_period_avg_ms,ads_tx_period_min_ms,ads_tx_period_max_ms,ads_tx_period_std_ms,"
             << "ads_tx_write_avg_ms,ads_tx_write_max_ms,ads_tx_cycle_avg_ms,ads_tx_cycle_max_ms,"
             << "ctrl_seq,ctrl_hz,ctrl_period_avg_ms,ctrl_period_min_ms,ctrl_period_max_ms,ctrl_period_std_ms,"
             << "ctrl_cycle_avg_ms,ctrl_cycle_max_ms\n";
        out_.flush();
    }

    const std::filesystem::path& path() const { return path_; }

    void append(const TimingSnapshot& ads_rx,
                const TimingSnapshot& ads_tx,
                const TimingSnapshot& control) {
        const auto now = std::chrono::system_clock::now();
        const int mode_id = g_current_mode.load(std::memory_order_relaxed);

        out_ << format_local_time(now, "%Y-%m-%d %H:%M:%S")
             << ',' << mode_to_string(mode_id)
             << ',' << mode_id
             << ',' << g_fsm_mode_count.load(std::memory_order_relaxed)
             << ',' << ads_rx.seq
             << ',' << ads_rx.hz
             << ',' << ads_rx.period_avg_ms
             << ',' << ads_rx.period_min_ms
             << ',' << ads_rx.period_max_ms
             << ',' << ads_rx.period_std_ms
             << ',' << ads_rx.read_avg_ms
             << ',' << ads_rx.read_max_ms
             << ',' << ads_rx.cycle_avg_ms
             << ',' << ads_rx.cycle_max_ms
             << ',' << ads_tx.seq
             << ',' << ads_tx.hz
             << ',' << ads_tx.period_avg_ms
             << ',' << ads_tx.period_min_ms
             << ',' << ads_tx.period_max_ms
             << ',' << ads_tx.period_std_ms
             << ',' << ads_tx.write_avg_ms
             << ',' << ads_tx.write_max_ms
             << ',' << ads_tx.cycle_avg_ms
             << ',' << ads_tx.cycle_max_ms
             << ',' << control.seq
             << ',' << control.hz
             << ',' << control.period_avg_ms
             << ',' << control.period_min_ms
             << ',' << control.period_max_ms
             << ',' << control.period_std_ms
             << ',' << control.cycle_avg_ms
             << ',' << control.cycle_max_ms
             << '\n';
        out_.flush();
    }

private:
    std::filesystem::path path_;
    std::ofstream out_;
};

void publish_thread_timing(ThreadTimingReport& report,
                           const TimingAccumulator& period_stats,
                           const TimingAccumulator& read_stats,
                           const TimingAccumulator& write_stats,
                           const TimingAccumulator& cycle_stats) {
    const double avg_period_ms = period_stats.avg_ms();
    report.hz.store(avg_period_ms > 0.0 ? (1000.0 / avg_period_ms) : 0.0);
    report.period_avg_ms.store(avg_period_ms);
    report.period_min_ms.store(
        std::isfinite(period_stats.min_ms) ? period_stats.min_ms : 0.0);
    report.period_max_ms.store(period_stats.max_ms);
    report.period_std_ms.store(period_stats.stddev_ms());
    report.read_avg_ms.store(read_stats.avg_ms());
    report.read_max_ms.store(read_stats.max_ms);
    report.write_avg_ms.store(write_stats.avg_ms());
    report.write_max_ms.store(write_stats.max_ms);
    report.cycle_avg_ms.store(cycle_stats.avg_ms());
    report.cycle_max_ms.store(cycle_stats.max_ms);
    report.seq.fetch_add(1, std::memory_order_relaxed);
}

void timing_logger_loop(const uint32_t timing_log_ms) {
    unsigned long long last_ads_rx_seq = 0;
    unsigned long long last_ads_tx_seq = 0;
    unsigned long long last_control_seq = 0;
    unsigned long long last_logged_ads_rx_seq = 0;
    unsigned long long last_logged_ads_tx_seq = 0;
    unsigned long long last_logged_control_seq = 0;
    TimingCsvLogger csv_logger;
    const auto sleep_interval = std::chrono::milliseconds(std::max<uint32_t>(50, timing_log_ms / 4));

    std::cout << "Timing CSV: " << csv_logger.path().string() << "\n";

    while (keepRunning.load()) {
        const auto ads_rx = load_timing_snapshot(g_ads_rx_timing_report);
        if (ads_rx.seq != 0 && ads_rx.seq != last_ads_rx_seq) {
            last_ads_rx_seq = ads_rx.seq;
            std::cout << "ADS RX timing: hz=" << ads_rx.hz
                      << " period_ms(avg/min/max/std)="
                      << ads_rx.period_avg_ms << "/"
                      << ads_rx.period_min_ms << "/"
                      << ads_rx.period_max_ms << "/"
                      << ads_rx.period_std_ms
                      << " read_ms(avg/max)="
                      << ads_rx.read_avg_ms << "/"
                      << ads_rx.read_max_ms
                      << " cycle_ms(avg/max)="
                      << ads_rx.cycle_avg_ms << "/"
                      << ads_rx.cycle_max_ms << "\n";
        }

        const auto ads_tx = load_timing_snapshot(g_ads_tx_timing_report);
        if (ads_tx.seq != 0 && ads_tx.seq != last_ads_tx_seq) {
            last_ads_tx_seq = ads_tx.seq;
            std::cout << "ADS TX timing: hz=" << ads_tx.hz
                      << " period_ms(avg/min/max/std)="
                      << ads_tx.period_avg_ms << "/"
                      << ads_tx.period_min_ms << "/"
                      << ads_tx.period_max_ms << "/"
                      << ads_tx.period_std_ms
                      << " write_ms(avg/max)="
                      << ads_tx.write_avg_ms << "/"
                      << ads_tx.write_max_ms
                      << " cycle_ms(avg/max)="
                      << ads_tx.cycle_avg_ms << "/"
                      << ads_tx.cycle_max_ms << "\n";
        }

        const auto control = load_timing_snapshot(g_control_timing_report);
        if (control.seq != 0 && control.seq != last_control_seq) {
            last_control_seq = control.seq;
            std::cout << "CTRL timing: hz=" << control.hz
                      << " period_ms(avg/min/max/std)="
                      << control.period_avg_ms << "/"
                      << control.period_min_ms << "/"
                      << control.period_max_ms << "/"
                      << control.period_std_ms
                      << " cycle_ms(avg/max)="
                      << control.cycle_avg_ms << "/"
                      << control.cycle_max_ms << "\n";
        }

        if (ads_rx.seq != 0 || ads_tx.seq != 0 || control.seq != 0) {
            if (ads_rx.seq != last_logged_ads_rx_seq || ads_tx.seq != last_logged_ads_tx_seq ||
                control.seq != last_logged_control_seq) {
                csv_logger.append(ads_rx, ads_tx, control);
                last_logged_ads_rx_seq = ads_rx.seq;
                last_logged_ads_tx_seq = ads_tx.seq;
                last_logged_control_seq = control.seq;
            }
        }

        std::this_thread::sleep_for(sleep_interval);
    }
}

int16_t motor_torque_to_iq(const double tau_motor_nm, const MotorSpec& spec) {
    if (spec.kt_nm_per_a <= 0.0 || spec.counts_per_amp <= 0.0 || spec.efficiency <= 0.0) {
        return 0;
    }

    const double iq_amp = tau_motor_nm / (spec.kt_nm_per_a * spec.efficiency);
    const double iq_count = iq_amp * spec.counts_per_amp;
    return static_cast<int16_t>(std::lround(std::clamp(iq_count, -2048.0, 2048.0)));
}

std::string trim_copy(std::string text) {
    const auto is_space = [](const unsigned char ch) { return std::isspace(ch) != 0; };
    text.erase(text.begin(), std::find_if(text.begin(), text.end(),
                                          [&](const unsigned char ch) { return !is_space(ch); }));
    text.erase(std::find_if(text.rbegin(), text.rend(),
                            [&](const unsigned char ch) { return !is_space(ch); })
                   .base(),
               text.end());
    return text;
}

std::vector<std::string> split_csv(const std::string& input) {
    std::vector<std::string> tokens;
    std::stringstream stream(input);
    std::string token;
    while (std::getline(stream, token, ',')) {
        token = trim_copy(token);
        if (!token.empty()) {
            tokens.push_back(token);
        }
    }
    return tokens;
}

void print_usage(const char* argv0) {
    std::cerr << "Usage: " << argv0 << " [ADS options] <urdf_path> [test_log_dir]\n"
              << "\nADS options:\n"
              << "  --ads                         optional compatibility flag; ADS is always used\n"
              << "  --ads-remote-ip <ip>         Beckhoff/Windows IP address\n"
              << "  --ads-remote-net-id <netid>  Beckhoff AMS Net ID\n"
              << "  --ads-local-net-id <netid>   local Ubuntu AMS Net ID\n"
              << "  --ads-port <port>            ADS port (default: 851)\n"
              << "  --ads-timeout-ms <ms>        ADS request timeout in milliseconds (default: 5000)\n"
              << "  --ads-feedback-plc-type <real|lreal|int16|uint16|int32|uint32>\n"
              << "  --ads-command-plc-type <real|lreal|int16|uint16|int32|uint32>\n"
              << "  --ads-reference-plc-type <real|lreal|int16|uint16|int32|uint32>\n"
              << "  --ads-gain-plc-type <real|lreal|int16|uint16|int32|uint32>\n"
              << "  --ads-command-source <torque|iq>\n"
              << "  --ads-pos-unit <rad|deg>     motor position feedback unit (default: rad)\n"
              << "  --ads-vel-unit <rad|deg>     motor velocity feedback unit (default: rad)\n"
              << "  --ads-motorstatus-prefix <sym>  PLC MotorStatus struct symbol (default: GVL_Status.MotorStatus)\n"
              << "  --ads-motorcmd-prefix <sym>     PLC MotorCmd struct symbol (default: GVL_Cmd.MotorCmd)\n"
              << "                                 (single ADS blob write path)\n"
              << "  --ads-status-bundle-symbol <sym> PLC MotorStatus struct symbol for blob read\n"
              << "  --ads-command-bundle-symbol <sym> PLC MotorCmd struct symbol for blob write\n"
              << "  --ads-pos-array-symbol <sym> PLC motor position array symbol\n"
              << "  --ads-pos-symbols <a[,b,c,d]> PLC motor position symbol(s); 1 item binds motor1 only\n"
              << "  --ads-vel-array-symbol <sym> PLC motor velocity array symbol\n"
              << "  --ads-vel-symbols <a[,b,c,d]> PLC motor velocity symbol(s); 1 item binds motor1 only\n"
              << "  --ads-cmd-array-symbol <sym> PLC motor command array symbol\n"
              << "  --ads-cmd-symbols <a[,b,c,d]> PLC motor command symbol(s); 1 item binds motor1 only\n"
              << "  --ads-enable-symbol <sym>    PLC enable BOOL symbol\n"
              << "  --ads-output-bundle-symbol <sym> PLC output bundle struct symbol\n"
              << "  --ads-pos-ref-array-symbol <sym> PLC motor position reference array symbol\n"
              << "  --ads-pos-ref-symbols <a[,b,c,d]> PLC motor position reference symbol(s); 1 item binds motor1 only\n"
              << "  --ads-vel-ref-array-symbol <sym> PLC motor velocity reference array symbol\n"
              << "  --ads-vel-ref-symbols <a[,b,c,d]> PLC motor velocity reference symbol(s); 1 item binds motor1 only\n"
              << "  --ads-kp-symbol <sym>        PLC scalar gain symbol for Kp\n"
              << "  --ads-kd-symbol <sym>        PLC scalar gain symbol for Kd\n"
              << "  --ads-read-only             skip PLC command writes and only perform feedback reads\n"
              << "  --ads-write-only            skip PLC feedback reads and only perform command writes\n"
              << "  --ads-timing-log-ms <ms>    print loop/read/write timing stats every N ms\n"
              << "  --ads-debug-refs            print packed motor ref values periodically\n"
#ifdef ADS_CONTROL_TEST_WITH_ROS2
              << "\nROS options:\n"
              << "  --ros-disable-publish       disable ROS 2 topic publishing thread\n"
              << "  --ros-publish-rate-hz <hz>  ROS 2 publish rate for mjustate/joint_states (default: 30)\n";
#else
              ;
#endif
}

ads_io::PlcType parse_plc_type(const std::string& text) {
    if (text == "real") return ads_io::PlcType::Real;
    if (text == "lreal") return ads_io::PlcType::LReal;
    if (text == "int16") return ads_io::PlcType::Int16;
    if (text == "uint16" || text == "uint") return ads_io::PlcType::UInt16;
    if (text == "int32" || text == "dint") return ads_io::PlcType::Int32;
    if (text == "uint32" || text == "udint") return ads_io::PlcType::UInt32;
    throw std::runtime_error("Invalid PLC type: " + text);
}

ads_io::AngleUnit parse_angle_unit(const std::string& text) {
    if (text == "rad") return ads_io::AngleUnit::Rad;
    if (text == "deg") return ads_io::AngleUnit::Deg;
    throw std::runtime_error("Invalid angle unit: " + text);
}

ads_io::CommandSource parse_command_source(const std::string& text) {
    if (text == "torque") return ads_io::CommandSource::Torque;
    if (text == "iq") return ads_io::CommandSource::Iq;
    throw std::runtime_error("Invalid command source: " + text);
}

void set_array_binding(ads_io::SymbolBinding& binding, const std::string& symbol) {
    binding.array_symbol = symbol;
    binding.symbols.clear();
}

void set_symbol_list(ads_io::SymbolBinding& binding, const std::vector<std::string>& symbols) {
    binding.array_symbol.clear();
    binding.symbols = symbols;
}

void set_single_symbol(ads_io::SymbolBinding& binding, const std::string& symbol) {
    binding.array_symbol.clear();
    binding.symbols = symbol.empty() ? std::vector<std::string>{} : std::vector<std::string>{symbol};
}

void apply_motorstatus_prefix(ads_io::Config& config, const std::string& prefix) {
    if (prefix.empty()) return;
    config.motor_status_bundle_symbol = prefix;
    config.motor_pos_symbols = ads_io::SymbolBinding{};
    config.motor_vel_symbols = ads_io::SymbolBinding{};
}

void apply_motorcmd_prefix(ads_io::Config& config, const std::string& prefix) {
    if (prefix.empty()) return;
    config.motor_command_bundle_symbol = prefix;
    config.motor_cmd_symbols = ads_io::SymbolBinding{};
    config.motor_pos_ref_symbols = ads_io::SymbolBinding{};
    config.motor_vel_ref_symbols = ads_io::SymbolBinding{};
    config.motor_enable_symbol.clear();
    config.motor_kp_symbol.clear();
    config.motor_kd_symbol.clear();
}

RuntimeOptions parse_runtime_options(int argc, char** argv) {
    RuntimeOptions options;
    std::vector<std::string> positional;

    const auto require_value = [&](const std::string& arg_name, int& index) -> std::string {
        if (index + 1 >= argc) {
            throw std::runtime_error("Missing value for argument: " + arg_name);
        }
        return argv[++index];
    };

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--ads") {
            continue;
        }
        if (arg == "--ads-debug-refs") {
            options.debug_refs = true;
            continue;
        }
#ifdef ADS_CONTROL_TEST_WITH_ROS2
        if (arg == "--ros-disable-publish") {
            options.ros_publish_enabled = false;
            continue;
        }
        if (arg == "--ros-publish-rate-hz") {
            options.ros.publish_rate_hz = std::stod(require_value(arg, i));
            continue;
        }
#endif
        if (arg == "--ads-read-only") {
            options.read_only = true;
            options.ads.read_only = true;
            continue;
        }
        if (arg == "--ads-write-only") {
            options.write_only = true;
            options.ads.write_only = true;
            continue;
        }
        if (arg == "--ads-timing-log-ms") {
            options.timing_log_ms = static_cast<uint32_t>(std::stoul(require_value(arg, i)));
            continue;
        }
        if (arg == "--ads-remote-ip") {
            options.ads.remote_ip = require_value(arg, i);
            continue;
        }
        if (arg == "--ads-remote-net-id") {
            options.ads.remote_net_id = require_value(arg, i);
            continue;
        }
        if (arg == "--ads-local-net-id") {
            options.ads.local_net_id = require_value(arg, i);
            continue;
        }
        if (arg == "--ads-port") {
            options.ads.port = static_cast<uint16_t>(std::stoul(require_value(arg, i)));
            continue;
        }
        if (arg == "--ads-timeout-ms") {
            options.ads.timeout_ms = static_cast<uint32_t>(std::stoul(require_value(arg, i)));
            continue;
        }
        if (arg == "--ads-feedback-plc-type") {
            options.ads.feedback_plc_type = parse_plc_type(require_value(arg, i));
            continue;
        }
        if (arg == "--ads-command-plc-type" || arg == "--ads-plc-type") {
            options.ads.command_plc_type = parse_plc_type(require_value(arg, i));
            continue;
        }
        if (arg == "--ads-reference-plc-type" || arg == "--ads-ref-plc-type") {
            options.ads.reference_plc_type = parse_plc_type(require_value(arg, i));
            continue;
        }
        if (arg == "--ads-gain-plc-type") {
            options.ads.gain_plc_type = parse_plc_type(require_value(arg, i));
            continue;
        }
        if (arg == "--ads-command-source" || arg == "--ads-source") {
            options.ads.command_source = parse_command_source(require_value(arg, i));
            continue;
        }
        if (arg == "--ads-pos-unit") {
            options.ads.motor_pos_unit = parse_angle_unit(require_value(arg, i));
            continue;
        }
        if (arg == "--ads-vel-unit") {
            options.ads.motor_vel_unit = parse_angle_unit(require_value(arg, i));
            continue;
        }
        if (arg == "--ads-motorstatus-prefix") {
            apply_motorstatus_prefix(options.ads, require_value(arg, i));
            continue;
        }
        if (arg == "--ads-motorcmd-prefix") {
            apply_motorcmd_prefix(options.ads, require_value(arg, i));
            continue;
        }
        if (arg == "--ads-status-bundle-symbol") {
            options.ads.motor_status_bundle_symbol = require_value(arg, i);
            continue;
        }
        if (arg == "--ads-command-bundle-symbol") {
            options.ads.motor_command_bundle_symbol = require_value(arg, i);
            continue;
        }
        if (arg == "--ads-pos-array-symbol") {
            set_array_binding(options.ads.motor_pos_symbols, require_value(arg, i));
            continue;
        }
        if (arg == "--ads-pos-symbols") {
            set_symbol_list(options.ads.motor_pos_symbols, split_csv(require_value(arg, i)));
            continue;
        }
        if (arg == "--ads-vel-array-symbol") {
            set_array_binding(options.ads.motor_vel_symbols, require_value(arg, i));
            continue;
        }
        if (arg == "--ads-vel-symbols") {
            set_symbol_list(options.ads.motor_vel_symbols, split_csv(require_value(arg, i)));
            continue;
        }
        if (arg == "--ads-cmd-array-symbol" || arg == "--ads-array-symbol") {
            set_array_binding(options.ads.motor_cmd_symbols, require_value(arg, i));
            continue;
        }
        if (arg == "--ads-cmd-symbols" || arg == "--ads-symbols") {
            set_symbol_list(options.ads.motor_cmd_symbols, split_csv(require_value(arg, i)));
            continue;
        }
        if (arg == "--ads-enable-symbol") {
            options.ads.motor_enable_symbol = require_value(arg, i);
            continue;
        }
        if (arg == "--ads-output-bundle-symbol" || arg == "--ads-bundle-symbol") {
            options.ads.motor_output_bundle_symbol = require_value(arg, i);
            continue;
        }
        if (arg == "--ads-pos-ref-array-symbol") {
            set_array_binding(options.ads.motor_pos_ref_symbols, require_value(arg, i));
            continue;
        }
        if (arg == "--ads-pos-ref-symbols") {
            set_symbol_list(options.ads.motor_pos_ref_symbols, split_csv(require_value(arg, i)));
            continue;
        }
        if (arg == "--ads-vel-ref-array-symbol") {
            set_array_binding(options.ads.motor_vel_ref_symbols, require_value(arg, i));
            continue;
        }
        if (arg == "--ads-vel-ref-symbols") {
            set_symbol_list(options.ads.motor_vel_ref_symbols, split_csv(require_value(arg, i)));
            continue;
        }
        if (arg == "--ads-kp-symbol") {
            options.ads.motor_kp_symbol = require_value(arg, i);
            continue;
        }
        if (arg == "--ads-kd-symbol") {
            options.ads.motor_kd_symbol = require_value(arg, i);
            continue;
        }
        if (!arg.empty() && arg[0] == '-') {
            throw std::runtime_error("Unknown option: " + arg);
        }
        positional.push_back(arg);
    }

    if (!positional.empty()) options.urdf_path_arg = positional[0];
    if (positional.size() >= 2) options.test_log_dir = positional[1];
    if (positional.size() > 2) {
        throw std::runtime_error("Too many positional arguments");
    }

    if (options.read_only && options.write_only) {
        throw std::runtime_error("--ads-read-only and --ads-write-only cannot be used together");
    }
    if (!options.write_only && options.ads.motor_status_bundle_symbol.empty() && options.ads.motor_pos_symbols.empty() && options.ads.motor_vel_symbols.empty()) {
        apply_motorstatus_prefix(options.ads, "GVL_Status.MotorStatus");
    }
    if (!options.read_only &&
        options.ads.motor_command_bundle_symbol.empty() &&
        options.ads.motor_cmd_symbols.empty() &&
        options.ads.motor_pos_ref_symbols.empty() &&
        options.ads.motor_vel_ref_symbols.empty() &&
        options.ads.motor_enable_symbol.empty() &&
        options.ads.motor_kp_symbol.empty() &&
        options.ads.motor_kd_symbol.empty() &&
        options.ads.motor_output_bundle_symbol.empty()) {
        apply_motorcmd_prefix(options.ads, "GVL_Cmd.MotorCmd");
    }
    if (options.ads.remote_ip.empty()) {
        throw std::runtime_error("--ads-remote-ip is required");
    }
    if (options.ads.remote_net_id.empty()) {
        throw std::runtime_error("--ads-remote-net-id is required");
    }
    if (!options.write_only && options.ads.motor_status_bundle_symbol.empty() && options.ads.motor_pos_symbols.empty()) {
        throw std::runtime_error("Specify --ads-status-bundle-symbol, --ads-pos-array-symbol, --ads-pos-symbols, or --ads-motorstatus-prefix");
    }
    if (!options.read_only &&
        options.ads.motor_command_bundle_symbol.empty() &&
        options.ads.motor_cmd_symbols.empty() && options.ads.motor_output_bundle_symbol.empty()) {
        throw std::runtime_error(
            "Specify --ads-command-bundle-symbol, --ads-cmd-array-symbol/--ads-cmd-symbols, --ads-output-bundle-symbol, or --ads-motorcmd-prefix");
    }

    return options;
}

std::string resolve_existing_path(const std::string& input_path, const char* argv0) {
    if (input_path.empty()) return {};

    std::error_code ec;
    const std::filesystem::path in(input_path);
    std::vector<std::filesystem::path> candidates {in};

    if (!in.is_absolute()) {
        candidates.push_back(std::filesystem::current_path(ec) / in);
        const std::filesystem::path exe_dir = std::filesystem::path(argv0).parent_path();
        candidates.push_back(exe_dir / in);
        candidates.push_back(exe_dir / ".." / in);
        candidates.push_back(exe_dir / ".." / ".." / in);
        if (input_path.rfind("../", 0) == 0) {
            candidates.push_back(in.relative_path());
        }
    }

    for (const auto& candidate : candidates) {
        const auto normalized = candidate.lexically_normal();
        if (std::filesystem::exists(normalized, ec)) {
            return std::filesystem::absolute(normalized, ec).string();
        }
    }
    return input_path;
}

std::array<MotorSpec, kMotorCount> build_motor_specs() {
    constexpr double kCountsPerAmp = 2048.0 / 33.0;
    return {{
        {g_motor_gear_ratio[0].load(), 3.46, kCountsPerAmp, 0.8},
        {g_motor_gear_ratio[1].load(), 3.46, kCountsPerAmp, 0.8},
        {g_motor_gear_ratio[2].load(), 3.46, kCountsPerAmp, 0.8},
        {g_motor_gear_ratio[3].load(), 3.46, kCountsPerAmp, 0.8},
    }};
}

void sync_motor_gear_ratios(const std::array<MotorSpec, kMotorCount>& specs) {
    for (size_t i = 0; i < specs.size(); ++i) {
        g_motor_gear_ratio[i].store(specs[i].gear_ratio);
    }
}

ads_io::Command build_ads_command(const std::array<MotorSpec, kMotorCount>& specs) {
    ads_io::Command command;

    const std::array<double, kMotorCount> tau_motor_nm = {
        mjustate.Sim.Tau.motor1,
        mjustate.Sim.Tau.motor2,
        mjustate.Sim.Tau.motor3,
        mjustate.Sim.Tau.motor4,
    };
    const std::array<double, kMotorCount> pos_ref_rad = {
        mjustate.Ref.Pos.motor1,
        mjustate.Ref.Pos.motor2,
        mjustate.Ref.Pos.motor3,
        mjustate.Ref.Pos.motor4,
    };
    const std::array<double, kMotorCount> vel_ref_rad = {
        mjustate.Ref.Vel.motor1,
        mjustate.Ref.Vel.motor2,
        mjustate.Ref.Vel.motor3,
        mjustate.Ref.Vel.motor4,
    };

    const bool hw_execute_enabled = g_hw_execute_enabled.load();
    command.enable = hw_execute_enabled;
    command.motor_kp = g_joint_kp[0].load();
    command.motor_kd = g_joint_kd[0].load();
    for (size_t i = 0; i < kMotorCount; ++i) {
        command.motor_torque_nm[i] = hw_execute_enabled ? tau_motor_nm[i] : 0.0;
        command.motor_iq[i] = hw_execute_enabled ? motor_torque_to_iq(tau_motor_nm[i], specs[i]) : 0;
        command.motor_pos_ref_rad[i] = pos_ref_rad[i];
        command.motor_vel_ref_rad[i] = vel_ref_rad[i];
    }

    return command;
}

void run_ads_read_loop(ads_io::AdsMotorIo& ads_client,
                       MjuJoint& joint_reader,
                       const uint32_t timing_log_ms) {
    auto next_loop_time = std::chrono::steady_clock::now();
    auto last_timing_log_time = next_loop_time;
    uint64_t last_sample_seq = 0;
    std::chrono::steady_clock::time_point last_sample_time {};
    TimingAccumulator period_stats;
    TimingAccumulator read_stats;
    TimingAccumulator cycle_stats;

    while (keepRunning.load()) {
        const auto cycle_start = std::chrono::steady_clock::now();
        const auto read_start = std::chrono::steady_clock::now();
        const auto ads_feedback = ads_client.ReadFeedback();
        const auto read_end = std::chrono::steady_clock::now();

        if (ads_feedback.sample_seq != 0 && ads_feedback.sample_seq != last_sample_seq) {
            if (last_sample_seq != 0) {
                period_stats.add(
                    std::chrono::duration<double, std::milli>(ads_feedback.sample_time - last_sample_time).count());
            }

            MotorFeedbackState feedback;
            feedback.motor_pos_rad = ads_feedback.motor_pos_rad;
            feedback.motor_vel_rad = ads_feedback.motor_vel_rad;
            feedback.pos_valid = ads_feedback.pos_valid;
            feedback.vel_valid = ads_feedback.vel_valid;
            joint_reader.UpdateFromMotorState(feedback, dt);

            read_stats.add(std::chrono::duration<double, std::milli>(read_end - read_start).count());
            cycle_stats.add(std::chrono::duration<double, std::milli>(read_end - cycle_start).count());
            last_sample_seq = ads_feedback.sample_seq;
            last_sample_time = ads_feedback.sample_time;
        }

        if (timing_log_ms > 0 && period_stats.count > 0 &&
            std::chrono::duration_cast<std::chrono::milliseconds>(read_end - last_timing_log_time).count() >=
                timing_log_ms) {
            publish_thread_timing(g_ads_rx_timing_report, period_stats, read_stats, TimingAccumulator{}, cycle_stats);
            period_stats.reset();
            read_stats.reset();
            cycle_stats.reset();
            last_timing_log_time = read_end;
        }

        next_loop_time += std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(dt));
        std::this_thread::sleep_until(next_loop_time);
    }
}

void run_ads_write_loop(ads_io::AdsMotorIo& ads_client,
                        const std::array<MotorSpec, kMotorCount>& specs,
                        const bool debug_refs,
                        const uint32_t timing_log_ms) {
    auto next_loop_time = std::chrono::steady_clock::now();
    auto prev_cycle_start = next_loop_time;
    auto last_timing_log_time = next_loop_time;
    size_t cycle_count = 0;
    TimingAccumulator period_stats;
    TimingAccumulator write_stats;
    TimingAccumulator cycle_stats;

    while (keepRunning.load()) {
        const auto cycle_start = std::chrono::steady_clock::now();
        if (cycle_count > 0) {
            period_stats.add(std::chrono::duration<double, std::milli>(cycle_start - prev_cycle_start).count());
        }

        const auto command = build_ads_command(specs);
        if (debug_refs && (cycle_count % 500 == 0)) {
            std::cout << "ADS ref pos = ["
                      << command.motor_pos_ref_rad[0] << ", "
                      << command.motor_pos_ref_rad[1] << ", "
                      << command.motor_pos_ref_rad[2] << ", "
                      << command.motor_pos_ref_rad[3] << "] vel = ["
                      << command.motor_vel_ref_rad[0] << ", "
                      << command.motor_vel_ref_rad[1] << ", "
                      << command.motor_vel_ref_rad[2] << ", "
                      << command.motor_vel_ref_rad[3] << "]\n";
        }

        const auto write_start = std::chrono::steady_clock::now();
        ads_client.WriteCommand(command);
        const auto write_end = std::chrono::steady_clock::now();

        write_stats.add(std::chrono::duration<double, std::milli>(write_end - write_start).count());
        cycle_stats.add(std::chrono::duration<double, std::milli>(write_end - cycle_start).count());

        if (timing_log_ms > 0 &&
            std::chrono::duration_cast<std::chrono::milliseconds>(write_end - last_timing_log_time).count() >=
                timing_log_ms) {
            publish_thread_timing(g_ads_tx_timing_report, period_stats, TimingAccumulator{}, write_stats,
                                  cycle_stats);
            period_stats.reset();
            write_stats.reset();
            cycle_stats.reset();
            last_timing_log_time = write_end;
        }

        prev_cycle_start = cycle_start;
        ++cycle_count;

        next_loop_time += std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(dt));
        std::this_thread::sleep_until(next_loop_time);
    }
}

}  // namespace

int main(int argc, char** argv) {
    RuntimeOptions options;
    try {
        options = parse_runtime_options(argc, argv);
    } catch (const std::exception& e) {
        std::cerr << e.what() << "\n";
        print_usage(argv[0]);
        return 1;
    }

    const std::string urdf_path_arg = options.urdf_path_arg;
    const std::string urdf_path = resolve_existing_path(urdf_path_arg, argv[0]);

    if (urdf_path_arg.empty()) {
        print_usage(argv[0]);
        return 1;
    }

    {
        std::error_code ec;
        if (!std::filesystem::exists(urdf_path, ec)) {
            std::cerr << "URDF not found: " << urdf_path_arg << "\n";
            std::cerr << "cwd: " << std::filesystem::current_path().string() << "\n";
            return 1;
        }
    }

    try {
        ControllerLoop controller_loop(urdf_path, options.test_log_dir);
        MjuJoint joint_reader;
        g_timing_log_ms.store(options.timing_log_ms, std::memory_order_relaxed);

        const auto specs = build_motor_specs();
        sync_motor_gear_ratios(specs);

        std::unique_ptr<ads_io::AdsMotorIo> ads_rx_client;
        if (!options.write_only) {
            auto rx_config = options.ads;
            rx_config.read_only = true;
            rx_config.write_only = false;
            ads_rx_client = std::make_unique<ads_io::AdsMotorIo>(rx_config);
        }

        std::unique_ptr<ads_io::AdsMotorIo> ads_tx_client;
        if (!options.read_only) {
            auto tx_config = options.ads;
            tx_config.read_only = false;
            tx_config.write_only = true;
            ads_tx_client = std::make_unique<ads_io::AdsMotorIo>(tx_config);
        }

#ifdef ADS_CONTROL_TEST_WITH_ROS2
        std::unique_ptr<MjuStatePublisher> ros_state_publisher;
#endif

        std::thread control_thread([&]() { controller_loop.Run(); });
        std::thread ads_rx_thread;
        std::thread ads_tx_thread;
        std::thread timing_logger_thread;
#ifdef ADS_CONTROL_TEST_WITH_ROS2
        std::thread ros_publish_thread;
#endif
        std::mutex ads_error_mutex;
        std::string ads_error_message;
        const auto report_ads_error = [&](const char* label, const std::exception& e) {
            std::lock_guard<std::mutex> lock(ads_error_mutex);
            if (ads_error_message.empty()) {
                ads_error_message = std::string(label) + ": " + e.what();
            }
            keepRunning.store(false);
        };

        if (options.timing_log_ms > 0) {
            timing_logger_thread = std::thread([&]() {
                try {
                    timing_logger_loop(options.timing_log_ms);
                } catch (const std::exception& e) {
                    report_ads_error("Timing logger error", e);
                }
            });
        }

#ifdef ADS_CONTROL_TEST_WITH_ROS2
        if (options.ros_publish_enabled) {
            ros_state_publisher = std::make_unique<MjuStatePublisher>(options.ros);
            ros_publish_thread = std::thread([&]() {
                try {
                    ros_state_publisher->Run();
                } catch (const std::exception& e) {
                    report_ads_error("ROS publish error", e);
                }
            });
        }
#endif

        if (ads_rx_client) {
            ads_rx_thread = std::thread([&]() {
                try {
                    run_ads_read_loop(*ads_rx_client, joint_reader, options.timing_log_ms);
                } catch (const std::exception& e) {
                    report_ads_error("ADS RX error", e);
                }
            });
        }
        if (ads_tx_client) {
            ads_tx_thread = std::thread([&]() {
                try {
                    run_ads_write_loop(*ads_tx_client, specs, options.debug_refs, options.timing_log_ms);
                } catch (const std::exception& e) {
                    report_ads_error("ADS TX error", e);
                }
            });
        }

        if (ads_rx_thread.joinable()) ads_rx_thread.join();
        if (ads_tx_thread.joinable()) ads_tx_thread.join();
        if (timing_logger_thread.joinable()) timing_logger_thread.join();
#ifdef ADS_CONTROL_TEST_WITH_ROS2
        if (ros_publish_thread.joinable()) ros_publish_thread.join();
#endif
        if (control_thread.joinable()) control_thread.join();

        if (!ads_error_message.empty()) {
            std::cerr << ads_error_message << "\n";
            return 2;
        }

        if (!options.read_only) {
            try {
                ads_tx_client->WriteZero();
            } catch (const std::exception& e) {
                std::cerr << "Failed to zero ADS outputs: " << e.what() << "\n";
            }
        }

        return 0;
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return 2;
    }
}
