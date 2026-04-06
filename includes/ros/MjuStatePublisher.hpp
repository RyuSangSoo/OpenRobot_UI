#ifndef ADS_ROS_MJU_STATE_PUBLISHER_HPP
#define ADS_ROS_MJU_STATE_PUBLISHER_HPP

#include "control/MjuRobotState.hpp"

#include <string>

struct RosPublisherOptions {
    double publish_rate_hz = 30.0;
    std::string node_name = "ads_state_publisher";
    std::string mju_state_topic = "ads/mju_state";
    std::string joint_state_sim_topic = "joint_states";
    std::string joint_state_ref_topic = "joint_states_ref";
    std::string motor_state_sim_topic = "motor_states";
    std::string motor_state_ref_topic = "motor_states_ref";
    std::string test_config_cmd_topic = "ads/test_config_cmd";
    std::string test_config_state_topic = "ads/test_config_state";
};

class MjuStatePublisher {
public:
    explicit MjuStatePublisher(RosPublisherOptions options);

    void Run();

private:
    RosPublisherOptions options_;
};

#endif
