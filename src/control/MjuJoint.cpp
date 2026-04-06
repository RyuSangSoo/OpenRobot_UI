#include "MjuJoint.hpp"
#include "RssController.hpp"
#include <algorithm>
#include <chrono>

bool MjuJoint::UpdateFromMotorState(const MotorFeedbackState& feedback, const double dt) {
    if (dt <= 0.0) {
        std::cerr << "UpdateFromMotorState: dt must be > 0" << std::endl;
        return false;
    }

    const auto now = std::chrono::steady_clock::now();

    bool all_positions_valid = true;
    for (size_t i = 0; i < motor_pos_rad_.size(); ++i) {
        // 기본값은 이전 샘플 유지다.
        // ADS read 가 일부 축에서 실패하거나 vel 심볼이 비어 있어도 제어 상태를 급격히 깨지 않게 한다.
        motor_pos_rad_[i] = prev_motor_pos_rad_[i];
        motor_vel_rad_[i] = prev_motor_vel_rad_[i];

        if (!feedback.pos_valid[i]) {
            // 위치 자체를 못 읽으면 속도는 완전히 0 으로 끊지 않고 약하게 감쇠시킨다.
            motor_vel_rad_[i] = prev_motor_vel_rad_[i] * 0.95;
            prev_motor_vel_rad_[i] = motor_vel_rad_[i];
            all_positions_valid = false;
            continue;
        }

        motor_pos_rad_[i] = feedback.motor_pos_rad[i];

        if (feedback.vel_valid[i]) {
            motor_vel_rad_[i] = feedback.motor_vel_rad[i];
        } else if (has_prev_motor_[i]) {
            // velocity ADS 심볼이 없을 때는 위치 차분으로 속도를 추정한다.
            // PLC task 주기와 Linux loop 주기가 완전히 같지 않을 수 있어 실제 경과시간을 다시 계산한다.
            double dt_used = dt;
            if (has_prev_time_[i]) {
                dt_used = std::chrono::duration<double>(now - prev_motor_time_[i]).count();
            }
            dt_used = std::max(1e-4, dt_used);

            const double raw_vel = (motor_pos_rad_[i] - prev_motor_pos_rad_[i]) / dt_used;
            // ADS 지연이나 샘플 튐이 들어오더라도 제어기가 바로 폭주하지 않게 속도 제한과 LPF 를 건다.
            constexpr double kMaxMotorVelRad = 80.0;
            constexpr double kVelLpfAlpha = 0.25;
            const double clamped = std::clamp(raw_vel, -kMaxMotorVelRad, kMaxMotorVelRad);
            motor_vel_rad_[i] =
                (1.0 - kVelLpfAlpha) * prev_motor_vel_rad_[i] + kVelLpfAlpha * clamped;
        } else {
            motor_vel_rad_[i] = 0.0;
        }

        prev_motor_pos_rad_[i] = motor_pos_rad_[i];
        prev_motor_vel_rad_[i] = motor_vel_rad_[i];
        has_prev_motor_[i] = true;
        prev_motor_time_[i] = now;
        has_prev_time_[i] = true;
    }

    const double r1 = std::max(1e-9, g_motor_gear_ratio[0].load());
    const double r2 = std::max(1e-9, g_motor_gear_ratio[1].load());
    const double r3 = std::max(1e-9, g_motor_gear_ratio[2].load());
    const double r4 = std::max(1e-9, g_motor_gear_ratio[3].load());
    // 모터축 -> 조인트축 역변환 (diffTrans의 역식)
    // m1 = (j1 + j2) * r1, m2 = -(j1 - j2) * r2
    // => j1 = 0.5*(m1/r1 - m2/r2), j2 = 0.5*(m1/r1 + m2/r2)
    mjustate.Sim.Pos.joint1 = 0.5 * ((motor_pos_rad_[0] / r1) - (motor_pos_rad_[1] / r2));
    mjustate.Sim.Pos.joint2 = 0.5 * ((motor_pos_rad_[0] / r1) + (motor_pos_rad_[1] / r2));
    mjustate.Sim.Pos.joint3 = motor_pos_rad_[2] / r3;
    mjustate.Sim.Pos.joint4 = motor_pos_rad_[3] / r4;

    mjustate.Sim.Vel.joint1 = 0.5 * ((motor_vel_rad_[0] / r1) - (motor_vel_rad_[1] / r2));
    mjustate.Sim.Vel.joint2 = 0.5 * ((motor_vel_rad_[0] / r1) + (motor_vel_rad_[1] / r2));
    mjustate.Sim.Vel.joint3 = motor_vel_rad_[2] / r3;
    mjustate.Sim.Vel.joint4 = motor_vel_rad_[3] / r4;

    // 디버깅/모니터링용 모터축 상태도 함께 저장 (rad, rad/s)
    mjustate.Sim.Pos.motor1 = motor_pos_rad_[0];
    mjustate.Sim.Pos.motor2 = motor_pos_rad_[1];
    mjustate.Sim.Pos.motor3 = motor_pos_rad_[2];
    mjustate.Sim.Pos.motor4 = motor_pos_rad_[3];

    mjustate.Sim.Vel.motor1 = motor_vel_rad_[0];
    mjustate.Sim.Vel.motor2 = motor_vel_rad_[1];
    mjustate.Sim.Vel.motor3 = motor_vel_rad_[2];
    mjustate.Sim.Vel.motor4 = motor_vel_rad_[3];

    return all_positions_valid;
}
