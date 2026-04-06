#include "ros/MjuStatePublisher.hpp"

#include "control/MjuRobot.hpp"
#include "control/RssController.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace {

constexpr std::array<const char*, 4> kJointNames = {"joint1", "joint2", "joint3", "joint4"};
constexpr std::array<const char*, 4> kMotorNames = {"motor1", "motor2", "motor3", "motor4"};

constexpr size_t kTestConfigFieldCount = 5;

bool NearlyEqual(const double lhs, const double rhs) {
    return std::abs(lhs - rhs) <= 1e-9;
}

void AppendAxisState(std::vector<double>& data, const decltype(MjuState {}.Sim.Pos)& value) {
    data.insert(data.end(), {value.joint1, value.joint2, value.joint3, value.joint4});
    data.insert(data.end(), {value.joint1_pre, value.joint2_pre, value.joint3_pre, value.joint4_pre});
    data.insert(data.end(), {value.joint1_init, value.joint2_init, value.joint3_init, value.joint4_init});
    data.insert(data.end(), {value.motor1, value.motor2, value.motor3, value.motor4});
}

void AppendPhaseState(std::vector<double>& data, const decltype(MjuState {}.Sim)& value) {
    AppendAxisState(data, value.Pos);
    AppendAxisState(data, value.Vel);
    AppendAxisState(data, value.Acc);
    AppendAxisState(data, value.Tau);

    data.insert(data.end(), {value.Tip_pos.x(), value.Tip_pos.y(), value.Tip_pos.z()});
    data.insert(data.end(), {value.Tip_vel.x(), value.Tip_vel.y(), value.Tip_vel.z()});
    data.insert(data.end(), {value.Tip_acc.x(), value.Tip_acc.y(), value.Tip_acc.z()});
    data.insert(data.end(), {value.Tip_tau.x(), value.Tip_tau.y(), value.Tip_tau.z()});

    data.insert(data.end(), {
        value.x_init,
        value.y_init,
        value.z_init,
        static_cast<double>(value.CoM_x),
        static_cast<double>(value.CoM_y),
        static_cast<double>(value.CoM_z),
        static_cast<double>(value.CoM_roll),
        static_cast<double>(value.CoM_pitch),
        static_cast<double>(value.CoM_yaw),
    });
}

std_msgs::msg::Float64MultiArray BuildMjuStateMessage(const MjuState& state) {
    std_msgs::msg::Float64MultiArray msg;
    msg.layout.dim.resize(1);
    msg.layout.dim[0].label = "sim_then_ref_flattened";
    msg.layout.dim[0].size = 170;
    msg.layout.dim[0].stride = 170;
    msg.data.reserve(170);
    AppendPhaseState(msg.data, state.Sim);
    AppendPhaseState(msg.data, state.Ref);
    return msg;
}

sensor_msgs::msg::JointState BuildJointStateMessage(
    const rclcpp::Time& stamp,
    const decltype(MjuState {}.Sim)& phase_state,
    const bool include_effort) {
    sensor_msgs::msg::JointState msg;
    msg.header.stamp = stamp;
    msg.name.assign(kJointNames.begin(), kJointNames.end());
    msg.position = {
        phase_state.Pos.joint1,
        phase_state.Pos.joint2,
        phase_state.Pos.joint3,
        phase_state.Pos.joint4,
    };
    msg.velocity = {
        phase_state.Vel.joint1,
        phase_state.Vel.joint2,
        phase_state.Vel.joint3,
        phase_state.Vel.joint4,
    };
    if (include_effort) {
        msg.effort = {
            phase_state.Tau.joint1,
            phase_state.Tau.joint2,
            phase_state.Tau.joint3,
            phase_state.Tau.joint4,
        };
    }
    return msg;
}

sensor_msgs::msg::JointState BuildMotorStateMessage(
    const rclcpp::Time& stamp,
    const decltype(MjuState {}.Sim)& phase_state) {
    sensor_msgs::msg::JointState msg;
    msg.header.stamp = stamp;
    msg.name.assign(kMotorNames.begin(), kMotorNames.end());
    msg.position = {
        phase_state.Pos.motor1,
        phase_state.Pos.motor2,
        phase_state.Pos.motor3,
        phase_state.Pos.motor4,
    };
    msg.velocity = {
        phase_state.Vel.motor1,
        phase_state.Vel.motor2,
        phase_state.Vel.motor3,
        phase_state.Vel.motor4,
    };
    msg.effort = {
        phase_state.Tau.motor1,
        phase_state.Tau.motor2,
        phase_state.Tau.motor3,
        phase_state.Tau.motor4,
    };
    return msg;
}

std_msgs::msg::Float64MultiArray BuildTestConfigStateMessage() {
    std_msgs::msg::Float64MultiArray msg;
    msg.layout.dim.resize(1);
    msg.layout.dim[0].label = "motor_index,amplitude_rad,frequency_hz,current_mode,change_counter";
    msg.layout.dim[0].size = kTestConfigFieldCount;
    msg.layout.dim[0].stride = kTestConfigFieldCount;
    msg.data = {
        static_cast<double>(std::clamp(g_test_active_motor_index.load(), 0, 4)),
        std::max(0.0, g_test_amp.load()),
        std::max(0.0, g_test_freq_hz.load()),
        static_cast<double>(g_current_mode.load()),
        static_cast<double>(g_test_joint_change_counter.load()),
    };
    return msg;
}

void ApplyTestConfigCommand(const std_msgs::msg::Float64MultiArray& msg, const rclcpp::Logger& logger) {
    const int current_motor = std::clamp(g_test_joint_index.load(), 0, 4);
    const double current_amp = std::max(0.0, g_test_amp.load());
    const double current_freq = std::max(0.0, g_test_freq_hz.load());

    int next_motor = current_motor;
    double next_amp = current_amp;
    double next_freq = current_freq;

    bool saw_valid_value = false;

    if (!msg.data.empty()) {
        if (std::isfinite(msg.data[0])) {
            next_motor = std::clamp(static_cast<int>(std::lround(msg.data[0])), 0, 4);
            saw_valid_value = true;
        } else {
            RCLCPP_WARN(logger, "Ignored test config motor_index because it is not finite");
        }
    }
    if (msg.data.size() >= 2) {
        if (std::isfinite(msg.data[1])) {
            next_amp = std::max(0.0, msg.data[1]);
            saw_valid_value = true;
        } else {
            RCLCPP_WARN(logger, "Ignored test config amplitude_rad because it is not finite");
        }
    }
    if (msg.data.size() >= 3) {
        if (std::isfinite(msg.data[2])) {
            next_freq = std::max(0.0, msg.data[2]);
            saw_valid_value = true;
        } else {
            RCLCPP_WARN(logger, "Ignored test config frequency_hz because it is not finite");
        }
    }

    if (!saw_valid_value) {
        RCLCPP_WARN(logger, "Ignored empty or invalid test config command. Expected [motor_index, amplitude_rad, frequency_hz].");
        return;
    }

    const bool changed =
        next_motor != current_motor ||
        !NearlyEqual(next_amp, current_amp) ||
        !NearlyEqual(next_freq, current_freq);
    if (!changed) {
        return;
    }

    g_test_joint_index.store(next_motor);
    g_test_amp.store(next_amp);
    g_test_freq_hz.store(next_freq);
    const unsigned long long new_counter = g_test_joint_change_counter.fetch_add(1) + 1;

    RCLCPP_INFO(
        logger,
        "Applied test config: motor_index=%d amplitude_rad=%.6f frequency_hz=%.6f change_counter=%llu",
        next_motor,
        next_amp,
        next_freq,
        new_counter);
}

}  // namespace

MjuStatePublisher::MjuStatePublisher(RosPublisherOptions options)
    : options_(std::move(options)) {}

void MjuStatePublisher::Run() {
    int argc = 0;
    char** argv = nullptr;
    if (!rclcpp::ok()) {
        rclcpp::init(argc, argv);
    }

    auto node = std::make_shared<rclcpp::Node>(options_.node_name);
    const auto qos = rclcpp::SensorDataQoS().keep_last(5);
    auto mju_state_pub = node->create_publisher<std_msgs::msg::Float64MultiArray>(
        options_.mju_state_topic, qos);
    auto joint_state_sim_pub = node->create_publisher<sensor_msgs::msg::JointState>(
        options_.joint_state_sim_topic, qos);
    auto joint_state_ref_pub = node->create_publisher<sensor_msgs::msg::JointState>(
        options_.joint_state_ref_topic, qos);
    auto motor_state_sim_pub = node->create_publisher<sensor_msgs::msg::JointState>(
        options_.motor_state_sim_topic, qos);
    auto motor_state_ref_pub = node->create_publisher<sensor_msgs::msg::JointState>(
        options_.motor_state_ref_topic, qos);
    auto test_config_state_pub = node->create_publisher<std_msgs::msg::Float64MultiArray>(
        options_.test_config_state_topic, rclcpp::QoS(1).reliable().transient_local());
    auto test_config_cmd_sub = node->create_subscription<std_msgs::msg::Float64MultiArray>(
        options_.test_config_cmd_topic,
        rclcpp::QoS(10).reliable(),
        [logger = node->get_logger()](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
            ApplyTestConfigCommand(*msg, logger);
        });
    (void)test_config_cmd_sub;

    const double publish_rate_hz = options_.publish_rate_hz > 0.0 ? options_.publish_rate_hz : 30.0;
    rclcpp::WallRate rate(publish_rate_hz);

    while (keepRunning.load() && rclcpp::ok()) {
        const rclcpp::Time stamp = node->get_clock()->now();
        const MjuState snapshot = mjustate;

        mju_state_pub->publish(BuildMjuStateMessage(snapshot));
        joint_state_sim_pub->publish(BuildJointStateMessage(stamp, snapshot.Sim, false));
        joint_state_ref_pub->publish(BuildJointStateMessage(stamp, snapshot.Ref, true));
        motor_state_sim_pub->publish(BuildMotorStateMessage(stamp, snapshot.Sim));
        motor_state_ref_pub->publish(BuildMotorStateMessage(stamp, snapshot.Ref));
        test_config_state_pub->publish(BuildTestConfigStateMessage());

        rclcpp::spin_some(node);
        rate.sleep();
    }

    if (rclcpp::ok()) {
        rclcpp::shutdown();
    }
}
