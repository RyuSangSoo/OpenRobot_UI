#ifndef MJUROBOT_HPP
#define MJUROBOT_HPP

#include "header.hpp"
#include "MjuRobotState.hpp"
#include "Locomotion.h"
#include "RssController.hpp"

extern std::atomic<bool> keepRunning;
void signalHandler(int signum);
void installSignalHandler();

class RssController;

class MjuRobot
{
private:
    std::array<PDController, 4> pd_controllers{};
    std::array<FrictionCompensator, 4> friction_compensators{};

public:
    MjuRobot(const std::string& urdf_path);
    ~MjuRobot();

    RigidBodyDynamics::Model model;

    Locomotion LOCO;

    void StateUpdate();

    RigidBodyDynamics::Math::VectorNd Q;
    RigidBodyDynamics::Math::VectorNd QDot;
    RigidBodyDynamics::Math::VectorNd QDDot;
    RigidBodyDynamics::Math::VectorNd Tau;
    
    RigidBodyDynamics::Math::VectorNd Q_Ref;
    RigidBodyDynamics::Math::VectorNd QDot_Ref;
    RigidBodyDynamics::Math::VectorNd QDDot_Ref;
    RigidBodyDynamics::Math::VectorNd Tau_Ref;

    RigidBodyDynamics::Math::Scalar mass;

    int dof;

    // 모델 관련 부분들 (역기구학, 정기구학, 패턴생성 등)
    Eigen::Vector3d getBodyPosition(const std::string& body_name);
    bool IKSolver(const unsigned int body_id, const Eigen::Vector3d& body_point_local, const Eigen::Vector3d& target_world,
                  const RigidBodyDynamics::Math::VectorNd& Q_init, RigidBodyDynamics::Math::VectorNd& Q_out, 
                  double lambda = 0.01, double step_tol = 1e-6, double pos_tol = 1e-4, unsigned int max_iter = 100);
    
    Eigen::Vector3d Frame_0;
    Eigen::Vector3d Frame_1;
    Eigen::Vector3d Frame_2;
    Eigen::Vector3d Frame_3;
    Eigen::Vector3d Frame_4;
    Eigen::Vector3d Frame_5;

    double l_0;
    double l_1;
    double l_2;
    double l_3;

    double Body_x;
    double Body_y;
    double Body_z;

    double Body_dx;
    double Body_dy;
    double Body_dz;

    double Body_ddx;
    double Body_ddy;
    double Body_ddz;

    double TaskPosition(double t, double T, double start, double end);
    double QuinticTraj(double t, double type);

    void JointParsing(RigidBodyDynamics::Math::VectorNd& vec, int type);
    void JointParsingOffset(RigidBodyDynamics::Math::VectorNd& vec, int type);

    void InitMode(double t_global, double dt);
    void ReadyMode(double t_global, double dt);
    void TestMode(double t_global, double dt);
    void StandMode(double t_global, double dt);
    void WalkingMode(double button_count, double x_joy, double y_joy, double yaw_joy, double imu_roll, double imu_pitch);
    void SwimmingMode(double t_global, double dt);
    void UpdateTorqueCommand(double dt);

    void PatternInit();
    void WalkingPattern(double Mode_count, double joyX, double joyY, double joyYaw, double imu_roll, double imu_pitch, double dt);
    
    Eigen::Vector3f FL_desired_walkreadypos_local = Eigen::Vector3f::Zero();
    Eigen::Vector3f FR_desired_walkreadypos_local = Eigen::Vector3f::Zero();
    Eigen::Vector3f ML_desired_walkreadypos_local = Eigen::Vector3f::Zero();
    Eigen::Vector3f MR_desired_walkreadypos_local = Eigen::Vector3f::Zero();
    Eigen::Vector3f RL_desired_walkreadypos_local = Eigen::Vector3f::Zero();
    Eigen::Vector3f RR_desired_walkreadypos_local = Eigen::Vector3f::Zero();

    Eigen::Matrix3d RotX(double th);
    Eigen::Matrix3d RotY(double th);
    Eigen::Matrix3d RotZ(double th);

    bool use_actual_pos = false;
    bool ik_success = false;
    bool ik_success_prev = false;
    bool stand_ref_latched = false;
    bool test_mode_initialized_ = false;
    bool test_switch_in_progress_ = false;
    int test_active_motor_index_ = 0;
    int test_pending_motor_index_ = 0;
    double test_phase_time_s_ = 0.0;
    double test_transition_elapsed_s_ = 0.0;
    double test_transition_duration_s_ = 0.0;
    double test_transition_start_offset_ = 0.0;
    double test_current_offset_ = 0.0;
    double test_last_amp_ = 0.0;
    double test_last_freq_hz_ = 0.0;
    std::array<double, 4> test_base_motor_ref_ {0.0, 0.0, 0.0, 0.0};

    RigidBodyDynamics::Math::VectorNd Q_Ref_prev;
    RigidBodyDynamics::Math::VectorNd QDot_Ref_prev;

    void diffTrans();
    void ResetTestModeState();
};



#endif
