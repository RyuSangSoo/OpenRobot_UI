#ifndef UI_ADS_VER_ADS_STATE_UI_HPP
#define UI_ADS_VER_ADS_STATE_UI_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <array>
#include <memory>
#include <string>
#include <vector>

namespace ui_ads_ver {

constexpr size_t kJointCount = 4;

struct JointStateData {
    bool received = false;
    std::array<double, kJointCount> position_rad {};
    std::array<double, kJointCount> velocity_rad_s {};
    std::array<double, kJointCount> effort {};
};

struct TestConfigData {
    bool received = false;
    int joint_index = 0;  // 0..3: motor1..motor4, 4: all
    double amplitude_rad = 0.25;
    double frequency_hz = 0.2;
    int current_mode = 0;
    unsigned long long change_counter = 0;
};

struct UiSnapshot {
    JointStateData sim;
    JointStateData ref;
    JointStateData motor_sim;
    JointStateData motor_ref;
    TestConfigData test_config;
    std::vector<double> mju_state;
    size_t sim_count = 0;
    size_t ref_count = 0;
    size_t motor_sim_count = 0;
    size_t motor_ref_count = 0;
    size_t mju_count = 0;
    size_t test_config_count = 0;
    bool joint_order_warning = false;
    std::string sim_topic = "/joint_states";
    std::string ref_topic = "/joint_states_ref";
    std::string motor_sim_topic = "/motor_states";
    std::string motor_ref_topic = "/motor_states_ref";
    std::string mju_topic = "/ads/mju_state";
    std::string test_config_state_topic = "/ads/test_config_state";
    std::string test_config_cmd_topic = "/ads/test_config_cmd";
};

class AdsStateUiNode : public rclcpp::Node {
public:
    AdsStateUiNode();

    UiSnapshot Snapshot() const;
    void PublishTestConfigCommand(int joint_index, double amplitude_rad, double frequency_hz);

private:
    static void FillOrderedJointData(const sensor_msgs::msg::JointState& msg,
                                     JointStateData& data,
                                     bool& joint_order_warning,
                                     const std::array<std::string, kJointCount>& expected_names);

    void OnSimJointState(const sensor_msgs::msg::JointState& msg);
    void OnRefJointState(const sensor_msgs::msg::JointState& msg);
    void OnMotorSimJointState(const sensor_msgs::msg::JointState& msg);
    void OnMotorRefJointState(const sensor_msgs::msg::JointState& msg);
    void OnMjuState(const std_msgs::msg::Float64MultiArray& msg);
    void OnTestConfigState(const std_msgs::msg::Float64MultiArray& msg);

    std::string sim_topic_;
    std::string ref_topic_;
    std::string motor_sim_topic_;
    std::string motor_ref_topic_;
    std::string mju_topic_;
    std::string test_config_state_topic_;
    std::string test_config_cmd_topic_;
    JointStateData sim_data_;
    JointStateData ref_data_;
    JointStateData motor_sim_data_;
    JointStateData motor_ref_data_;
    TestConfigData test_config_;
    std::vector<double> mju_state_;
    size_t sim_count_ = 0;
    size_t ref_count_ = 0;
    size_t motor_sim_count_ = 0;
    size_t motor_ref_count_ = 0;
    size_t mju_count_ = 0;
    size_t test_config_count_ = 0;
    bool joint_order_warning_ = false;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sim_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr ref_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr motor_sim_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr motor_ref_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr mju_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr test_config_state_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr test_config_cmd_pub_;
};

int RunAdsStateUi(int argc, char** argv);

}  // namespace ui_ads_ver

#endif
