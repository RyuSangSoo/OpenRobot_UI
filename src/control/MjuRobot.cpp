#include "MjuRobot.hpp"
#include "RssController.hpp"
#include <array>

using namespace std;
using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;
using namespace RigidBodyDynamics::Utils;

std::atomic<bool> keepRunning{true};

void signalHandler(int sig) {
    std::cout << "\nInterrupt signal (" << sig << ") received.\n";
    keepRunning = false;
}

void installSignalHandler() {
    std::signal(SIGINT, signalHandler);
}

MjuRobot::MjuRobot(const std::string& urdf_path) {
    if (!Addons::URDFReadFromFile(urdf_path.c_str(), &model, true)) {
        cerr << "Error loading model: " << urdf_path << endl;
        exit(1);
    }

    model.gravity = Vector3d(0, 0, -9.81);

    dof             = model.q_size;

    Q               = RigidBodyDynamics::Math::VectorNd::Zero(dof);
    QDot            = RigidBodyDynamics::Math::VectorNd::Zero(dof);
    QDDot           = RigidBodyDynamics::Math::VectorNd::Zero(dof);
    Tau             = RigidBodyDynamics::Math::VectorNd::Zero(dof);

    Q_Ref           = RigidBodyDynamics::Math::VectorNd::Zero(dof);
    QDot_Ref        = RigidBodyDynamics::Math::VectorNd::Zero(dof);
    QDDot_Ref       = RigidBodyDynamics::Math::VectorNd::Zero(dof);
    Tau_Ref         = RigidBodyDynamics::Math::VectorNd::Zero(dof);
    Q_Ref_prev      = RigidBodyDynamics::Math::VectorNd::Zero(dof);
    QDot_Ref_prev   = RigidBodyDynamics::Math::VectorNd::Zero(dof);

    Frame_0         = getBodyPosition("base_link");
    Frame_1         = getBodyPosition("LEG1_Link");
    Frame_2         = getBodyPosition("LEG2_Link");
    Frame_3         = getBodyPosition("LEG3_Link");
    Frame_4         = getBodyPosition("LEG4_Link");
    Frame_5         = getBodyPosition("EE_Link");

    l_0             = (Frame_1 - Frame_2).norm();
    l_1             = (Frame_2 - Frame_3).norm();
    l_2             = (Frame_3 - Frame_4).norm();
    l_3             = (Frame_4 - Frame_5).norm();

}

MjuRobot::~MjuRobot() {
    cout << "FINISH MODEL" << endl;
}

void MjuRobot::StateUpdate() {
    // std::cout << "State Update" << std::endl;
    // Q[5]    = 1.57;
    Q[6]    = mjustate.Sim.Pos.joint1;
    Q[7]    = mjustate.Sim.Pos.joint2;
    Q[8]    = mjustate.Sim.Pos.joint3;
    Q[9]    = mjustate.Sim.Pos.joint4;

    QDot[6]    = mjustate.Sim.Vel.joint1;
    QDot[7]    = mjustate.Sim.Vel.joint2;
    QDot[8]    = mjustate.Sim.Vel.joint3;
    QDot[9]    = mjustate.Sim.Vel.joint4;

    mjustate.Sim.Tip_pos = RotZ(90 * D2R) * (CalcBodyToBaseCoordinates(model, Q, model.GetBodyId("EE_Link"), RigidBodyDynamics::Math::Vector3d(0.0, 0.0, 0.0), true));
    // mjustate.Sim.Tip_pos = (CalcBodyToBaseCoordinates(model, Q, model.GetBodyId("EE_Link"), RigidBodyDynamics::Math::Vector3d(0.0, 0.0, 0.0), true));
    // mjustate.Ref.Tip_pos = RotZ(90 * D2R) * (CalcBodyToBaseCoordinates(model, Q_Ref, model.GetBodyId("EE_Link"), RigidBodyDynamics::Math::Vector3d(0.0, 0.0, 0.0), true));

}

// 각 링크의 Body좌표계 기준 frame 좌표
Eigen::Vector3d MjuRobot::getBodyPosition(const std::string& body_name) {
    
    unsigned int body_id = model.GetBodyId(body_name.c_str());

    Eigen::Vector3d getpos = CalcBaseToBodyCoordinates(model, VectorNd::Zero(model.dof_count), body_id, Vector3d::Zero(), true);

    return getpos;
}


bool MjuRobot::IKSolver(const unsigned int body_id, const Eigen::Vector3d& body_point_local, const Eigen::Vector3d& target_world,
                        const RigidBodyDynamics::Math::VectorNd& Q_init, RigidBodyDynamics::Math::VectorNd& Q_out, 
                        double lambda, double step_tol, double pos_tol, unsigned int max_iter) {
    
    RigidBodyDynamics::Math::VectorNd q = Q_init;
    RigidBodyDynamics::Math::VectorNd q_prev = Q_init;   // 🟢 이전 valid joint 저장

    Eigen::Vector3d target_local = RotZ(90 * D2R).transpose() * target_world;

    for (unsigned int iter = 0; iter < max_iter; ++iter) {
        
        // 매 iteration 시작시 현재 q를 백업
        q_prev = q;     // 🟢 업데이트 이전의 q를 저장해 둔다

        // 1. 현재 Forward Kinematics 계산
        Eigen::Vector3d pos_current = CalcBodyToBaseCoordinates(model, q, body_id, body_point_local, true);

        // 2. TaskSpac 오차 계산
        Eigen::Vector3d err = target_local - pos_current;
        // ||err|| < pos_tol이면 종료 IK 성공
        if (err.norm() < pos_tol) {
            Q_out = q;
            return true;
        }

        // 3. Jacobian 계산
        RigidBodyDynamics::Math::MatrixNd Jacobian(3, dof);
        Jacobian.setZero();
        CalcPointJacobian(model, q, body_id, body_point_local, Jacobian, true);

        // 4. Damped Least Squares (DLS 계산)
        Eigen::MatrixXd JTJ = Jacobian.transpose() * Jacobian;
        Eigen::MatrixXd A = JTJ + lambda * lambda * Eigen::MatrixXd::Identity(dof, dof);
        Eigen::VectorXd b = Jacobian.transpose() * err;

        // 5. 선형 시스템 풀어서 delt q 계산 dq = A^(-1)*b
        Eigen::VectorXd dq = A.ldlt().solve(b);

        // 6. dq 크기 체크

        if (dq.norm() < step_tol) {
            Q_out = q;
            return true;
        }

        q += dq;

        if (q[9] > 0.0) {          // elbow-down 방향이면
            q[9] = 0.0;            // 경계까지 강제 clamp
        }

        q[7] = 0.0;

    }

    Q_out = q_prev;

    std::cout << "Inverse Kinematics Failed" << std::endl;

    return false;
}

double MjuRobot::TaskPosition(double t, double T, double start, double end) {
    double delta = end - start;
    return start + delta * 0.5 * (1 - cos(M_PI * t/ T));
}

double MjuRobot::QuinticTraj(double t, double type) {

    Eigen::VectorXd coef3[4], coef4[4];

    for (int i = 0; i < 3; i++) {
        
        coef3[i].resize(6);
        coef4[i].resize(6);
    }

    coef3[0] << 1.1781,  0, 0, -6.9813, 6.9813,  -1.8617;
    coef3[1] << -1.1781, 0, 0, 0,       0,       0;
    coef3[2] << -1.1781, 0, 0, 2.9452,  -2.2089, 0.4418;

    coef4[0] << 0.7854, 0, 0, 5.8178,  -5.8178, 1.5514;
    coef4[1] << 2.7489, 0, 0, -7.6277, 7.6277,  -2.0341;
    coef4[2] << 0.1745, 0, 0, 0.7636,  -0.5727, 0.1145;

    Eigen::VectorXd* coef = nullptr;

    switch ((int)type)
    {
        case 0: coef = &coef3[0]; break;
        case 1: coef = &coef4[0]; break;
        case 2: coef = &coef3[1]; break;
        case 3: coef = &coef4[1]; break;
        case 4: coef = &coef3[2]; break;
        case 5: coef = &coef4[2]; break;

        default:
            std::cerr << "QuinticTraj: Invalid type = " << type << std::endl;
            return 0.0;
    }

    return (*coef)(0)
         + (*coef)(1) * t
         + (*coef)(2) * pow(t, 2)
         + (*coef)(3) * pow(t, 3)
         + (*coef)(4) * pow(t, 4)
         + (*coef)(5) * pow(t, 5);

    
}

void MjuRobot::JointParsing(RigidBodyDynamics::Math::VectorNd& vec, int type) {
    switch (type)
    {
    case 0:
        mjustate.Ref.Pos.joint1 = vec[0+6];
        mjustate.Ref.Pos.joint2 = vec[1+6];
        mjustate.Ref.Pos.joint3 = vec[2+6];
        mjustate.Ref.Pos.joint4 = vec[3+6];

        break;

    case 1:
        mjustate.Ref.Vel.joint1 = vec[0+6];
        mjustate.Ref.Vel.joint2 = vec[1+6];
        mjustate.Ref.Vel.joint3 = vec[2+6];
        mjustate.Ref.Vel.joint4 = vec[3+6];

        break;

    case 2:
        mjustate.Ref.Acc.joint1 = vec[0+6];
        mjustate.Ref.Acc.joint2 = vec[1+6];
        mjustate.Ref.Acc.joint3 = vec[2+6];
        mjustate.Ref.Acc.joint4 = vec[3+6];

        break;
    }
}

void MjuRobot::JointParsingOffset(RigidBodyDynamics::Math::VectorNd& vec, int type) {
    switch (type)
    {
    case 0:
        mjustate.Ref.Pos.joint1 = vec[0+6] + joint1_offset;
        mjustate.Ref.Pos.joint2 = vec[1+6] + joint2_offset;
        mjustate.Ref.Pos.joint3 = vec[2+6] + joint3_offset;
        mjustate.Ref.Pos.joint4 = vec[3+6] + joint4_offset;

        break;

    case 1:
        mjustate.Ref.Vel.joint1 = vec[0+6];
        mjustate.Ref.Vel.joint2 = vec[1+6];
        mjustate.Ref.Vel.joint3 = vec[2+6];
        mjustate.Ref.Vel.joint4 = vec[3+6];

        break;

    case 2:
        mjustate.Ref.Acc.joint1 = vec[0+6];
        mjustate.Ref.Acc.joint2 = vec[1+6];
        mjustate.Ref.Acc.joint3 = vec[2+6];
        mjustate.Ref.Acc.joint4 = vec[3+6];

        break;
    }
}

void MjuRobot::InitMode(double t_global, double dt) {

    std::vector<RigidBodyDynamics::Math::SpatialVector> f_ext(model.mBodies.size(), RigidBodyDynamics::Math::SpatialVector::Zero());

    double t_des = 6.0;

    bool interp_done = t_global >= t_des;
    double t_clamped = interp_done ? t_des : t_global;

    // mjustate.Ref.Pos.joint1 = TaskPosition(t_clamped, t_des, mjustate.Sim.Pos.joint1_init, 0);
    // mjustate.Ref.Pos.joint2 = TaskPosition(t_clamped, t_des, mjustate.Sim.Pos.joint2_init, 0);
    // mjustate.Ref.Pos.joint3 = TaskPosition(t_clamped, t_des, mjustate.Sim.Pos.joint3_init, 0);
    // mjustate.Ref.Pos.joint4 = TaskPosition(t_clamped, t_des, mjustate.Sim.Pos.joint4_init, 0);

    Q_Ref[6] = TaskPosition(t_clamped, t_des, mjustate.Sim.Pos.joint1_init, 0);
    Q_Ref[7] = TaskPosition(t_clamped, t_des, mjustate.Sim.Pos.joint2_init, 0);
    Q_Ref[8] = TaskPosition(t_clamped, t_des, mjustate.Sim.Pos.joint3_init, 0);
    Q_Ref[9] = TaskPosition(t_clamped, t_des, mjustate.Sim.Pos.joint4_init, 0);

    JointParsing(Q_Ref, 0);
}

void MjuRobot::ReadyMode(double t_global, double dt) {

    std::vector<RigidBodyDynamics::Math::SpatialVector> f_ext(model.mBodies.size(), RigidBodyDynamics::Math::SpatialVector::Zero());

    double t_des = 6.0;

    bool interp_done = t_global >= t_des;
    double t_clamped = interp_done ? t_des : t_global;

    // mjustate.Ref.Tip_pos[0] = TaskPosition(t_clamped, t_des, mjustate.Sim.x_init, 0.8);
    // mjustate.Ref.Tip_pos[1] = TaskPosition(t_clamped, t_des, mjustate.Sim.y_init, 0);
    // mjustate.Ref.Tip_pos[2] = TaskPosition(t_clamped, t_des, mjustate.Sim.z_init, 0.1);

    mjustate.Ref.Tip_pos[0] = TaskPosition(t_clamped, t_des, mjustate.Sim.x_init, 0);
    mjustate.Ref.Tip_pos[1] = TaskPosition(t_clamped, t_des, mjustate.Sim.y_init, 0.8);
    mjustate.Ref.Tip_pos[2] = TaskPosition(t_clamped, t_des, mjustate.Sim.z_init, 0.1);

    // // Eigen::Vector3d target_local = RotZ(90*D2R).transpose() * mjustate.Ref.Tip_pos;

    // // std::cout << "target_local : " << target_local.transpose() << std::endl;

    // ik_success = IKSolver(model.GetBodyId("EE_Link"), RigidBodyDynamics::Math::Vector3d(0.0, 0.0, 0.0), mjustate.Ref.Tip_pos,
    //                            Q, Q_Ref, 0.01, 1e-6, 1e-4, 100);

    // if (!ik_success) {
    //     std::cout << "[ReadyMode] IK failed!" << std::endl;
    //     return;
    // }

    // mjustate.Ref.Tip_pos[0] = TaskPosition(t_clamped, t_des, mjustate.Sim.x_init, 0);
    // mjustate.Ref.Tip_pos[1] = TaskPosition(t_clamped, t_des, mjustate.Sim.y_init, 0.8);
    // mjustate.Ref.Tip_pos[2] = TaskPosition(t_clamped, t_des, mjustate.Sim.z_init, 0.1);

    // Eigen::Vector3d target_local = RotZ(90*D2R).transpose() * mjustate.Ref.Tip_pos;

    // std::cout << "target_local : " << target_local.transpose() << std::endl;

    Q_Ref[6] = TaskPosition(t_clamped, t_des, mjustate.Sim.Pos.joint1_init, -1.239807 + joint1_offset);
    Q_Ref[7] = TaskPosition(t_clamped, t_des, mjustate.Sim.Pos.joint2_init, -0.0 + joint2_offset);
    Q_Ref[8] = TaskPosition(t_clamped, t_des, mjustate.Sim.Pos.joint3_init, -0.00169 + joint3_offset);
    Q_Ref[9] = TaskPosition(t_clamped, t_des, mjustate.Sim.Pos.joint4_init, -2.087373+ joint4_offset);
    ////////////////////////////////////////////////////////////////////////////////////

    JointParsing(Q_Ref, 0);
    // JointParsingOffset(Q_Ref, 0);
}

void MjuRobot::StandMode(double t_global, double dt) {

    double t_des = 2.0;

    bool interp_done = t_global >= t_des;
    double t_clamped = interp_done ? t_des : t_global;

    if (!interp_done) {
        stand_ref_latched = false;
    }
    if (interp_done && stand_ref_latched) {
        JointParsing(Q_Ref, 0);
        return;
    }

    // mjustate.Ref.Tip_pos[0] = TaskPosition(t_clamped, t_des, mjustate.Sim.x_init, 0);
    // mjustate.Ref.Tip_pos[1] = TaskPosition(t_clamped, t_des, mjustate.Sim.y_init, 1.0);
    // mjustate.Ref.Tip_pos[2] = TaskPosition(t_clamped, t_des, mjustate.Sim.z_init, -0.4);

    mjustate.Ref.Tip_pos[0] = TaskPosition(t_clamped, t_des, mjustate.Ref.x_init, 0);
    mjustate.Ref.Tip_pos[1] = TaskPosition(t_clamped, t_des, mjustate.Ref.y_init, 1.0);
    mjustate.Ref.Tip_pos[2] = TaskPosition(t_clamped, t_des, mjustate.Ref.z_init, -0.4);

    ik_success = IKSolver(model.GetBodyId("EE_Link"), RigidBodyDynamics::Math::Vector3d(0.0, 0.0, 0.0), mjustate.Ref.Tip_pos,
                               Q, Q_Ref, 0.01, 1e-6, 1e-4, 100);

    if (!ik_success) {
        std::cout << "[StandMode] IK failed!" << std::endl;
        return;
    }
    if (interp_done) {
        stand_ref_latched = true;
    }

    ////////////////////////////////////////////////////////////////////////////////////


    JointParsing(Q_Ref, 0);
    // JointParsingOffset(Q_Ref, 0);
}

void MjuRobot::TestMode(double t_global, double dt) {
    (void)t_global;
    const int requested_motor_idx = std::max(0, std::min(4, g_test_joint_index.load()));
    const double amp = std::max(0.0, g_test_amp.load());
    const double freq_hz = std::max(0.0, g_test_freq_hz.load());
    const double r1 = std::max(1e-9, g_motor_gear_ratio[0].load());
    const double r2 = std::max(1e-9, g_motor_gear_ratio[1].load());
    const double r3 = std::max(1e-9, g_motor_gear_ratio[2].load());
    const double r4 = std::max(1e-9, g_motor_gear_ratio[3].load());
    const auto capture_base_motor_ref = [&]() {
        test_base_motor_ref_[0] = mjustate.Sim.Pos.motor1;
        test_base_motor_ref_[1] = mjustate.Sim.Pos.motor2;
        test_base_motor_ref_[2] = mjustate.Sim.Pos.motor3;
        test_base_motor_ref_[3] = mjustate.Sim.Pos.motor4;
    };

    if (!test_mode_initialized_) {
        capture_base_motor_ref();
        test_mode_initialized_ = true;
        test_switch_in_progress_ = false;
        test_active_motor_index_ = requested_motor_idx;
        test_pending_motor_index_ = requested_motor_idx;
        test_phase_time_s_ = 0.0;
        test_transition_elapsed_s_ = 0.0;
        test_transition_duration_s_ = 0.0;
        test_transition_start_offset_ = 0.0;
        test_current_offset_ = 0.0;
        test_last_amp_ = amp;
        test_last_freq_hz_ = freq_hz;
    }

    const bool waveform_changed =
        std::abs(amp - test_last_amp_) > 1e-9 ||
        std::abs(freq_hz - test_last_freq_hz_) > 1e-9;
    if (waveform_changed && !test_switch_in_progress_) {
        capture_base_motor_ref();
        test_phase_time_s_ = 0.0;
        test_current_offset_ = 0.0;
    }
    test_last_amp_ = amp;
    test_last_freq_hz_ = freq_hz;

    if (!test_switch_in_progress_ && requested_motor_idx != test_active_motor_index_) {
        test_switch_in_progress_ = true;
        test_pending_motor_index_ = requested_motor_idx;
        test_transition_elapsed_s_ = 0.0;
        test_transition_start_offset_ = test_current_offset_;
        const double half_period_s = freq_hz > 1e-6 ? (0.5 / freq_hz) : 0.5;
        test_transition_duration_s_ = std::clamp(half_period_s, 0.15, 1.0);
    } else if (test_switch_in_progress_) {
        test_pending_motor_index_ = requested_motor_idx;
    }

    if (test_switch_in_progress_) {
        test_transition_elapsed_s_ += std::max(dt, 0.0);
        const double duration_s = std::max(1e-6, test_transition_duration_s_);
        const double alpha = std::min(1.0, test_transition_elapsed_s_ / duration_s);
        const double smooth = 1.0 - (alpha * alpha * (3.0 - 2.0 * alpha));
        test_current_offset_ = test_transition_start_offset_ * smooth;
        if (alpha >= 1.0 || std::abs(test_current_offset_) < 1e-6) {
            test_current_offset_ = 0.0;
            test_switch_in_progress_ = false;
            test_active_motor_index_ = test_pending_motor_index_;
            capture_base_motor_ref();
            test_phase_time_s_ = 0.0;
        }
    } else {
        test_phase_time_s_ += std::max(dt, 0.0);
        const double w = 2.0 * M_PI * freq_hz;
        test_current_offset_ = amp * std::sin(w * test_phase_time_s_);
    }

    g_test_active_motor_index.store(test_active_motor_index_);

    double motor_ref1 = test_base_motor_ref_[0];
    double motor_ref2 = test_base_motor_ref_[1];
    double motor_ref3 = test_base_motor_ref_[2];
    double motor_ref4 = test_base_motor_ref_[3];

    if (test_active_motor_index_ == 4) {
        motor_ref1 += test_current_offset_;
        motor_ref2 += test_current_offset_;
        motor_ref3 += test_current_offset_;
        motor_ref4 += test_current_offset_;
    } else {
        switch (test_active_motor_index_) {
            case 0: motor_ref1 += test_current_offset_; break;
            case 1: motor_ref2 += test_current_offset_; break;
            case 2: motor_ref3 += test_current_offset_; break;
            default: motor_ref4 += test_current_offset_; break;
        }
    }

    Q_Ref[6] = 0.5 * ((motor_ref1 / r1) - (motor_ref2 / r2));
    Q_Ref[7] = 0.5 * ((motor_ref1 / r1) + (motor_ref2 / r2));
    Q_Ref[8] = motor_ref3 / r3;
    Q_Ref[9] = motor_ref4 / r4;

    JointParsing(Q_Ref, 0);
}

void MjuRobot::ResetTestModeState() {
    test_mode_initialized_ = false;
    test_switch_in_progress_ = false;
    test_active_motor_index_ = std::max(0, std::min(4, g_test_joint_index.load()));
    test_pending_motor_index_ = test_active_motor_index_;
    test_phase_time_s_ = 0.0;
    test_transition_elapsed_s_ = 0.0;
    test_transition_duration_s_ = 0.0;
    test_transition_start_offset_ = 0.0;
    test_current_offset_ = 0.0;
    test_last_amp_ = std::max(0.0, g_test_amp.load());
    test_last_freq_hz_ = std::max(0.0, g_test_freq_hz.load());
    test_base_motor_ref_ = {0.0, 0.0, 0.0, 0.0};
    g_test_active_motor_index.store(test_active_motor_index_);
}

void MjuRobot::WalkingMode(double button_count, double x_joy, double y_joy, double yaw_joy, double imu_roll, double imu_pitch) {

    WalkingPattern(button_count, x_joy, y_joy, yaw_joy, imu_roll, imu_pitch, dt);

    ik_success = IKSolver(model.GetBodyId("EE_Link"), RigidBodyDynamics::Math::Vector3d(0.0, 0.0, 0.0), mjustate.Ref.Tip_pos,
                               Q, Q_Ref, 0.01, 1e-6, 1e-4, 100);

    if (!ik_success) {
        std::cout << "[WalkingMode] IK failed!" << std::endl;
        return;
    }

    if (ik_success && (ik_success != ik_success_prev)) {
        std::cout << "###########################" << std::endl;
        std::cout << "[WalkingMode] WALKING READY " << std::endl;
        std::cout << "###########################" << std::endl;
    }

    ik_success_prev = ik_success;

    ////////////////////////////////////////////////////////////////////////////////////


    JointParsing(Q_Ref, 0);
    // JointParsingOffset(Q_Ref, 0);
}

void MjuRobot::SwimmingMode(double t_global, double dt) {
    
    double t_des = 2.0;

    bool interp_done = t_global >= t_des;
    double t_clamped = interp_done ? t_des : t_global;

    Q_Ref[6] = TaskPosition(t_clamped, t_des, mjustate.Sim.Pos.joint1_init, 1.1781 + joint1_offset);
    Q_Ref[7] = TaskPosition(t_clamped, t_des, mjustate.Sim.Pos.joint2_init, 0.0000439486 + joint2_offset);
    Q_Ref[8] = TaskPosition(t_clamped, t_des, mjustate.Sim.Pos.joint3_init, 0.000199569 + joint3_offset);
    Q_Ref[9] = TaskPosition(t_clamped, t_des, mjustate.Sim.Pos.joint4_init, -0.7854 + joint4_offset);

    double total_time = 5;

    double speed_scale = 0.5; // 0.5 = 2배 느림, 2.0 = 2배 빠름

    double t_period = fmod(t_global - t_des, total_time * (1.0 / speed_scale));

    // Quintic 함수에 들어갈 시간 스케일링
    double t_scaled;

    // if ((t >= 0.0 && t < 1.5)) {
    //     Q_Ref[6] = QuinticTraj(t, 0);
    //     Q_Ref[9] = -QuinticTraj(t, 1);
    // }

    // else if ((t >= 1.5 && t < 3.0)) {
    //     Q_Ref[6] = QuinticTraj(t - 1.5, 2);
    //     Q_Ref[9] = -QuinticTraj(t - 1.5, 3);
    // }

    // if ((t >= 3.0 && t <= total_time)) {
    //     Q_Ref[6] = QuinticTraj(t - 3, 4);
    //     Q_Ref[9] = -QuinticTraj(t - 3, 5);
    // }

    // 1) 0.0 ~ 1.5 구간
    if (t_period >= 0.0 && t_period < 1.5 / speed_scale) {
        t_scaled = t_period * speed_scale;    // 0~1.5 로 압축
        Q_Ref[6] = QuinticTraj(t_scaled, 0) + joint1_offset;
        Q_Ref[9] = -QuinticTraj(t_scaled, 1) + joint4_offset;
    }

    // 2) 1.5 ~ 3.0 구간
    else if (t_period >= 1.5 / speed_scale && t_period < 3.0 / speed_scale) {
        t_scaled = (t_period - (1.5 / speed_scale)) * speed_scale;  // 새로운 스케일 기반 offset
        Q_Ref[6] = QuinticTraj(t_scaled, 2) + joint1_offset;
        Q_Ref[9] = -QuinticTraj(t_scaled, 3) + joint4_offset;
    }

    // 3) 3.0 ~ total_time 구간
    else if (t_period >= 3.0 / speed_scale && t_period <= total_time / speed_scale) {
        t_scaled = (t_period - (3.0 / speed_scale)) * speed_scale;
        Q_Ref[6] = QuinticTraj(t_scaled, 4) + joint1_offset;
        Q_Ref[9] = -QuinticTraj(t_scaled, 5) + joint4_offset;
    }

    JointParsing(Q_Ref, 0);
    // JointParsingOffset(Q_Ref, 0);
}

void MjuRobot::PatternInit() {

    FL_desired_walkreadypos_local << 0.0, 1.0, -0.4;
    FR_desired_walkreadypos_local << 0.0, 1.0, -0.4;
    ML_desired_walkreadypos_local << 0.0, 1.0, -0.4;
    MR_desired_walkreadypos_local << 0.0, 1.0, -0.4;
    RL_desired_walkreadypos_local << 0.0, 1.0, -0.4;
    RR_desired_walkreadypos_local << 0.0, 1.0, -0.4;


    // 보행 패턴 생성에 필요한 매개변수들을 초기화합니다. 아래는 국민대의 3점 보행 시 시간/스윙높이 정보입니다. 필요에 맞게 적어주세요.
    float swing_time = 0.5;     // swing time (3점 지지 시간) [s]
    float stance_time = 0.1;   // stance time (6점 지지 시간) [s]
    float swing_height = 0.2;   // swing height (다리 드는 높이) [m]

    float thread_time = 0.002;

    LOCO.Locomotion_Initialize(FL_desired_walkreadypos_local,
                               FR_desired_walkreadypos_local,
                               ML_desired_walkreadypos_local,
                               MR_desired_walkreadypos_local,
                               RL_desired_walkreadypos_local,
                               RR_desired_walkreadypos_local,
                               swing_time,  
                               stance_time,  
                               swing_height,  
                               thread_time);
}

void MjuRobot::WalkingPattern(double Mode_count, double joyX, double joyY, double joyYaw, double imu_roll, double imu_pitch, double dt) {

    LOCO.parameter[0].joystick.count = Mode_count;      // 0: 정지, 1: 3점지지, 2: 4점지지, 3: 5점지지

    LOCO.parameter[0].joystick.InputData << joyX, joyY, joyYaw;

    // LOCO.parameter[0].joystick.count = 1;

    // LOCO.parameter[0].joystick.InputData << 1, 0, 0;

    LOCO.Locomotion_Trajectory_Generator(LOCO.parameter[0],
                                         mjustate.Ref.Tip_pos.cast<float>(),
                                         Eigen::Vector3f::Zero(),
                                         Eigen::Vector3f::Zero(),
                                         Eigen::Vector3f::Zero(),
                                         Eigen::Vector3f::Zero(),
                                         Eigen::Vector3f::Zero(),
                                         imu_roll,
                                         imu_pitch,
                                         use_actual_pos
    );

    // LOCO.Locomotion_Trajectory_Generator(LOCO.parameter[0],
    //                                      Eigen::Vector3f::Zero(),
    //                                      Eigen::Vector3f::Zero(),
    //                                      Eigen::Vector3f::Zero(),
    //                                      Eigen::Vector3f::Zero(),
    //                                      Eigen::Vector3f::Zero(),
    //                                      Eigen::Vector3f::Zero(),
    //                                      imu_roll,
    //                                      imu_pitch,
    //                                      use_actual_pos
    // );

    mjustate.Ref.CoM_roll   = LOCO.parameter[0].CoM_Ref.G_Ori_Traj(AXIS_ROLL);
    mjustate.Ref.CoM_pitch  = LOCO.parameter[0].CoM_Ref.G_Ori_Traj(AXIS_PITCH);
    mjustate.Ref.CoM_yaw    = LOCO.parameter[0].CoM_Ref.G_Ori_Traj(AXIS_YAW);

    mjustate.Ref.CoM_x      = LOCO.parameter[0].CoM_Ref.G_Pos_Traj(AXIS_X);
    mjustate.Ref.CoM_y      = LOCO.parameter[0].CoM_Ref.G_Pos_Traj(AXIS_Y);
    mjustate.Ref.CoM_z      = LOCO.parameter[0].CoM_Ref.G_Pos_Traj(AXIS_Z);

    mjustate.Ref.Tip_pos = (LOCO.Z_Rot(LOCO.parameter[0].CoM_Ref.G_Ori_Traj(2), 3) * (LOCO.parameter[0].FL_Foot_Ref.G_Pos_Traj - LOCO.parameter[0].CoM_Ref.G_Pos_Traj)).cast<double>();
    // mjustate.Ref.Tip_pos = (LOCO.Z_Rot(1.57, 3) * (LOCO.parameter[0].FL_Foot_Ref.G_Pos_Traj - LOCO.parameter[0].CoM_Ref.G_Pos_Traj)).cast<double>();
}

Eigen::Matrix3d MjuRobot::RotX(double th) {
    
    Eigen::Matrix3d rotx;
    
    rotx << 1, 0, 0,
            0, cos(th), -sin(th),
            0, sin(th), cos(th);

    return rotx;
}

Eigen::Matrix3d MjuRobot::RotY(double th) {
    
    Eigen::Matrix3d roty;
    
    roty << cos(th), 0, sin(th),
            0, 1, 0,
            -sin(th), 0, cos(th);

    return roty;
}

Eigen::Matrix3d MjuRobot::RotZ(double th) {
    
    Eigen::Matrix3d rotz;
    
    rotz << cos(th), -sin(th), 0,
            sin(th), cos(th), 0,
            0, 0, 1;

    return rotz;
}

void MjuRobot::UpdateTorqueCommand(double dt) {
    if (dt <= 0.0) return;

    constexpr std::array<int, 4> kIdx = {6, 7, 8, 9};
    const std::array<double, 4> kp = {
        g_joint_kp[0].load(), g_joint_kp[1].load(),
        g_joint_kp[2].load(), g_joint_kp[3].load()};
    const std::array<double, 4> kd = {
        g_joint_kd[0].load(), g_joint_kd[1].load(),
        g_joint_kd[2].load(), g_joint_kd[3].load()};
    const std::array<double, 4> fv = {
        g_joint_fv[0].load(), g_joint_fv[1].load(),
        g_joint_fv[2].load(), g_joint_fv[3].load()};
    const std::array<double, 4> fc = {
        g_joint_fc[0].load(), g_joint_fc[1].load(),
        g_joint_fc[2].load(), g_joint_fc[3].load()};
    const std::array<double, 4> fs = {
        g_joint_fs[0].load(), g_joint_fs[1].load(),
        g_joint_fs[2].load(), g_joint_fs[3].load()};
    const std::array<double, 4> vs = {
        g_joint_vs[0].load(), g_joint_vs[1].load(),
        g_joint_vs[2].load(), g_joint_vs[3].load()};
    const std::array<double, 4> alpha = {
        g_joint_alpha[0].load(), g_joint_alpha[1].load(),
        g_joint_alpha[2].load(), g_joint_alpha[3].load()};
    const std::array<double, 4> eps = {
        g_joint_eps[0].load(), g_joint_eps[1].load(),
        g_joint_eps[2].load(), g_joint_eps[3].load()};
    const bool friction_ff_enable = g_friction_ff_enable.load();
    const double tau_limit = 300.0;

    // 현재 관절 상태를 동역학 벡터로 구성
    Q.setZero();
    QDot.setZero();
    Q[kIdx[0]] = mjustate.Sim.Pos.joint1;
    Q[kIdx[1]] = mjustate.Sim.Pos.joint2;
    Q[kIdx[2]] = mjustate.Sim.Pos.joint3;
    Q[kIdx[3]] = mjustate.Sim.Pos.joint4;
    QDot[kIdx[0]] = mjustate.Sim.Vel.joint1;
    QDot[kIdx[1]] = mjustate.Sim.Vel.joint2;
    QDot[kIdx[2]] = mjustate.Sim.Vel.joint3;
    QDot[kIdx[3]] = mjustate.Sim.Vel.joint4;

    // 모드에서 설정한 Q_Ref로부터 속도/가속도 참조를 수치 미분으로 생성
    for (int i = 0; i < 4; ++i) {
        const int idx = kIdx[i];
        QDot_Ref[idx] = (Q_Ref[idx] - Q_Ref_prev[idx]) / dt;
        QDDot_Ref[idx] = (QDot_Ref[idx] - QDot_Ref_prev[idx]) / dt;
    }

    std::vector<RigidBodyDynamics::Math::SpatialVector> f_ext(
        model.mBodies.size(), RigidBodyDynamics::Math::SpatialVector::Zero());
    RigidBodyDynamics::InverseDynamics(model, Q_Ref, QDot_Ref, QDDot_Ref, Tau_Ref, &f_ext);

    mjustate.Ref.Tau.joint1 = Tau_Ref[kIdx[0]];
    mjustate.Ref.Tau.joint2 = Tau_Ref[kIdx[1]];
    mjustate.Ref.Tau.joint3 = Tau_Ref[kIdx[2]];
    mjustate.Ref.Tau.joint4 = Tau_Ref[kIdx[3]];

    const auto clamp_tau = [tau_limit](double v) {
        return std::max(-tau_limit, std::min(tau_limit, v));
    };

    for (int i = 0; i < 4; ++i) {
        pd_controllers[i].setGains(kp[i], kd[i]);
        friction_compensators[i].setParams(fv[i], fc[i], fs[i], vs[i], alpha[i], eps[i]);
    }

    const double tau_pd1 =
        pd_controllers[0].pdcalculate(Q_Ref[kIdx[0]], QDot_Ref[kIdx[0]], Q[kIdx[0]], QDot[kIdx[0]]);
    const double tau_pd2 =
        pd_controllers[1].pdcalculate(Q_Ref[kIdx[1]], QDot_Ref[kIdx[1]], Q[kIdx[1]], QDot[kIdx[1]]);
    const double tau_pd3 =
        pd_controllers[2].pdcalculate(Q_Ref[kIdx[2]], QDot_Ref[kIdx[2]], Q[kIdx[2]], QDot[kIdx[2]]);
    const double tau_pd4 =
        pd_controllers[3].pdcalculate(Q_Ref[kIdx[3]], QDot_Ref[kIdx[3]], Q[kIdx[3]], QDot[kIdx[3]]);

    const double tau_fric1 = friction_ff_enable ? friction_compensators[0].calculate(QDot[kIdx[0]]) : 0.0;
    const double tau_fric2 = friction_ff_enable ? friction_compensators[1].calculate(QDot[kIdx[1]]) : 0.0;
    const double tau_fric3 = friction_ff_enable ? friction_compensators[2].calculate(QDot[kIdx[2]]) : 0.0;
    const double tau_fric4 = friction_ff_enable ? friction_compensators[3].calculate(QDot[kIdx[3]]) : 0.0;

    g_joint_tau_pd[0].store(tau_pd1);
    g_joint_tau_pd[1].store(tau_pd2);
    g_joint_tau_pd[2].store(tau_pd3);
    g_joint_tau_pd[3].store(tau_pd4);
    g_joint_tau_fric[0].store(tau_fric1);
    g_joint_tau_fric[1].store(tau_fric2);
    g_joint_tau_fric[2].store(tau_fric3);
    g_joint_tau_fric[3].store(tau_fric4);

    // const double tau1 = clamp_tau(Tau_Ref[kIdx[0]] + tau_pd1 + tau_fric1);
    // const double tau2 = clamp_tau(Tau_Ref[kIdx[1]] + tau_pd2 + tau_fric2);
    // const double tau3 = clamp_tau(Tau_Ref[kIdx[2]] + tau_pd3 + tau_fric3);
    // const double tau4 = clamp_tau(Tau_Ref[kIdx[3]] + tau_pd4 + tau_fric4);

    const double tau1 = clamp_tau(tau_pd1);
    const double tau2 = clamp_tau(tau_pd2);
    const double tau3 = clamp_tau(tau_pd3);
    const double tau4 = clamp_tau(tau_pd4);

    mjustate.Sim.Tau.joint1 = tau1;
    mjustate.Sim.Tau.joint2 = tau2;
    mjustate.Sim.Tau.joint3 = tau3;
    mjustate.Sim.Tau.joint4 = tau4;

    mjustate.Ref.Vel.joint1 = QDot_Ref[kIdx[0]];
    mjustate.Ref.Vel.joint2 = QDot_Ref[kIdx[1]];
    mjustate.Ref.Vel.joint3 = QDot_Ref[kIdx[2]];
    mjustate.Ref.Vel.joint4 = QDot_Ref[kIdx[3]];
    mjustate.Ref.Acc.joint1 = QDDot_Ref[kIdx[0]];
    mjustate.Ref.Acc.joint2 = QDDot_Ref[kIdx[1]];
    mjustate.Ref.Acc.joint3 = QDDot_Ref[kIdx[2]];
    mjustate.Ref.Acc.joint4 = QDDot_Ref[kIdx[3]];

    Q_Ref_prev = Q_Ref;
    QDot_Ref_prev = QDot_Ref;
}

void MjuRobot::diffTrans() {
    const double r1 = std::max(1e-9, g_motor_gear_ratio[0].load());
    const double r2 = std::max(1e-9, g_motor_gear_ratio[1].load());
    const double r3 = std::max(1e-9, g_motor_gear_ratio[2].load());
    const double r4 = std::max(1e-9, g_motor_gear_ratio[3].load());

    // 위치/속도 변환: 조인트축 -> 모터축
    mjustate.Ref.Pos.motor1 = (mjustate.Ref.Pos.joint1 + mjustate.Ref.Pos.joint2) * r1;
    mjustate.Ref.Pos.motor2 = -(mjustate.Ref.Pos.joint1 - mjustate.Ref.Pos.joint2) * r2;
    mjustate.Ref.Pos.motor3 = (mjustate.Ref.Pos.joint3) * r3;
    mjustate.Ref.Pos.motor4 = (mjustate.Ref.Pos.joint4) * r4;

    mjustate.Ref.Vel.motor1 = (mjustate.Ref.Vel.joint1 + mjustate.Ref.Vel.joint2) * r1;
    mjustate.Ref.Vel.motor2 = -(mjustate.Ref.Vel.joint1 - mjustate.Ref.Vel.joint2) * r2;
    mjustate.Ref.Vel.motor3 = (mjustate.Ref.Vel.joint3) * r3;
    mjustate.Ref.Vel.motor4 = (mjustate.Ref.Vel.joint4) * r4;

    // mjustate.Sim.Pos.motor1 = (mjustate.Sim.Pos.joint1 + mjustate.Sim.Pos.joint2) * r1;
    // mjustate.Sim.Pos.motor2 = -(mjustate.Sim.Pos.joint1 - mjustate.Sim.Pos.joint2) * r2;
    // mjustate.Sim.Pos.motor3 = (mjustate.Sim.Pos.joint3) * r3;
    // mjustate.Sim.Pos.motor4 = (mjustate.Sim.Pos.joint4) * r4;

    // mjustate.Sim.Vel.motor1 = (mjustate.Sim.Vel.joint1 + mjustate.Sim.Vel.joint2) * r1;
    // mjustate.Sim.Vel.motor2 = -(mjustate.Sim.Vel.joint1 - mjustate.Sim.Vel.joint2) * r2;
    // mjustate.Sim.Vel.motor3 = (mjustate.Sim.Vel.joint3) * r3;
    // mjustate.Sim.Vel.motor4 = (mjustate.Sim.Vel.joint4) * r4;

    // 토크 변환: 조인트축 -> 모터축 (파워 보존 기반 역변환)
    // m1 = (j1 + j2) * r1, m2 = (j1 - j2) * r2
    // => tau_m1 = 0.5*(tau_j1/r1 + tau_j2/r1), tau_m2 = 0.5*(tau_j1/r2 - tau_j2/r2)
    mjustate.Ref.Tau.motor1 = 0.5 * ((mjustate.Ref.Tau.joint1 / r1) + (mjustate.Ref.Tau.joint2 / r1));
    mjustate.Ref.Tau.motor2 = -0.5 * ((mjustate.Ref.Tau.joint1 / r2) - (mjustate.Ref.Tau.joint2 / r2));
    mjustate.Ref.Tau.motor3 = (mjustate.Ref.Tau.joint3) / r3;
    mjustate.Ref.Tau.motor4 = (mjustate.Ref.Tau.joint4) / r4;

    mjustate.Sim.Tau.motor1 = 0.5 * ((mjustate.Sim.Tau.joint1 / r1) + (mjustate.Sim.Tau.joint2 / r1));
    mjustate.Sim.Tau.motor2 = -0.5 * ((mjustate.Sim.Tau.joint1 / r2) - (mjustate.Sim.Tau.joint2 / r2));
    mjustate.Sim.Tau.motor3 = (mjustate.Sim.Tau.joint3) / r3;
    mjustate.Sim.Tau.motor4 = (mjustate.Sim.Tau.joint4) / r4;

}
