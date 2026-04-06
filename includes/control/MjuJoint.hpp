#ifndef MJUROSNODE_HPP
#define MJUROSNODE_HPP

#include "header.hpp"
#include "MjuRobotState.hpp"
#include <array>
#include <chrono>

struct MotorFeedbackState {
    std::array<double, 4> motor_pos_rad {0.0, 0.0, 0.0, 0.0};
    std::array<double, 4> motor_vel_rad {0.0, 0.0, 0.0, 0.0};
    std::array<bool, 4> pos_valid {false, false, false, false};
    std::array<bool, 4> vel_valid {false, false, false, false};
};

// 모터축 상태를 읽어 조인트 상태로 변환해 mjustate에 반영
class MjuJoint {
public:
    // dt: 제어 주기 [s]
    bool UpdateFromMotorState(const MotorFeedbackState& feedback, double dt);

private:
    std::array<double, 4> motor_pos_rad_ {0.0, 0.0, 0.0, 0.0};
    std::array<double, 4> motor_vel_rad_ {0.0, 0.0, 0.0, 0.0};
    std::array<double, 4> prev_motor_vel_rad_ {0.0, 0.0, 0.0, 0.0};
    std::array<double, 4> prev_motor_pos_rad_ {0.0, 0.0, 0.0, 0.0};
    std::array<bool, 4> has_prev_motor_ {false, false, false, false};
    std::array<std::chrono::steady_clock::time_point, 4> prev_motor_time_ {};
    std::array<bool, 4> has_prev_time_ {false, false, false, false};
};

#endif
