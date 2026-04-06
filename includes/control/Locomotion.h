/*
 * ============================================================
 * Locomotion Class Header
 * 
 * Robotics & Control Lab.
 * 
 * File                 : locomotion.h
 * Author               : BK Cho
 *
 * First developer      : Jeong Hwan. Jang
 * Second developer     : WON SUK. JI
 * 
 * Update date          : 2024. 05. 27 by Jeong Hwan. Jang
 * ============================================================
 */

#ifndef LOCOMOTION_H
#define LOCOMOTION_H

// #include "CRobot/CRobot.h"

//*
#include <Eigen/Dense>
#define PI              3.141592653589793238462
#define R2D             180./PI
#define D2R             PI/180.
#define GRAVITY         9.81

#define JOINT_NUM       24

#define AXIS_X          0
#define AXIS_Y          1
#define AXIS_Z          2

#define AXIS_ROLL       0
#define AXIS_PITCH      1
#define AXIS_YAW        2

#define GEAR_RATIO      36

#define JOINT_KP_HY           1500
#define JOINT_KP_HP           200
#define JOINT_KP_HR           200
#define JOINT_KP_KN           200

#define JOINT_KD_HY           5
#define JOINT_KD_HP           4
#define JOINT_KD_HR           4
#define JOINT_KD_KN           4

#define tasktime        0.001
#define onesecScale     1000

#define NUM_OF_LEG      6
#define NUM_OF_DoF      4
#define NUM_OF_POS      3
#define NUM_OF_ROT_Ang  3

#define horizontal_length 1

using namespace std;

//* Eigen library
using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::MatrixXf;
using Eigen::VectorXf;

using Eigen::Matrix2f;
using Eigen::Matrix3f;
using Eigen::Matrix4f;
// using Eigen::Matrix5f;
// using Eigen::Matrix6f;
using Matrix5f = Eigen::Matrix<float, 5, 5>;
using Matrix6f = Eigen::Matrix<float, 6, 6>;


using Eigen::Vector2f;
using Eigen::Vector3f;
using Eigen::Vector4f;
// using Eigen::Vector5f;
// using Eigen::Vector6f;
using Vector5f = Eigen::Matrix<float, 5, 1>;
using Vector6f = Eigen::Matrix<float, 6, 1>;

typedef Eigen::Matrix<float, JOINT_NUM, 1> Vector24f;
typedef Eigen::Matrix<float, JOINT_NUM + 6, 1> Vector30f;
typedef Eigen::Matrix<float, 3*NUM_OF_LEG, 1> Vector18f;

//*

typedef enum {
    
    NO_WALKING,
    TRIPOD_GAIT,
    RIPPLE_GAIT,
    WAVE_GAIT,

}GAIT_TYPE;

typedef enum {
    NO_SWING,
    FLMRRL_SWING, FRMLRR_SWING,                                         //* Tripod
    FLRR_SWING, MRRL_SWING, FRML_SWING,                                 //* Ripple
    FL_SWING, FR_SWING, ML_SWING, MR_SWING, RL_SWING, RR_SWING,         //* Wave 

}SWING_LEG;

typedef enum {
    
    NO_PHASE,
    SWING_PHASE,
    STANCE_PHASE,

}_PHASE;

typedef struct Time{
    
    float t_Stance;
    float t_Swing;
    float t_Period;
    float t_Gait;
    float t_Footmotion;
    float t_Gait_prev;

    float task_time;

}TIME;

typedef struct TripodGait{
    Vector3f SpeedScale;
    Vector3f LimitAcceleration;
}TRIPOD;

typedef struct RippleGait{
    Vector3f SpeedScale;
    Vector3f LimitAcceleration;
}RIPPLE;

typedef struct WaveGait{
    Vector3f SpeedScale;
    Vector3f LimitAcceleration;
}WAVE;

typedef struct Joystick{
    
    // unsigned int InputData;
    unsigned int count;

    Vector3f InputData;
}JOYSTICK;

typedef struct Gait{
    TRIPOD tripod;
    RIPPLE ripple;
    WAVE wave;
    SWING_LEG swingleg;

    unsigned int Type;
    unsigned int Type_Target;
    unsigned int Phase;
    
    unsigned int Swing_Leg;
    unsigned int Swing_Leg_prev;

    bool Stance_ON_Flag;
    bool Swing_ON_Flag;
    bool Swing_UP_Flag;
    bool Swing_DOWN_Flag;
    
    bool Check_Flag;
    bool ON_Flag;
    
    Vector3f CoM_MoveSize;
    float CoM_FB_MoveSize;
    float CoM_LR_MoveSize;
    float BASE_TurnSize;
    
    float Foot_FBstepSize;
    float Foot_LRstepSize;
    float Foot_TurnSize;

    float FootHeight;

    //*
    float BaseHeight;

    Vector3f CoM_InputData              = Vector3f::Ones(3);
    Vector3f Target_Speed;
    Vector3f Pre_Moving_Speed;
    Vector3f Moving_Speed;
    Vector3f Turn_Speed;
    Vector3f Prev_Target_Speed;

}GAIT;

typedef struct CoMRef{
    Vector3f G_Pos_Traj;
    Vector3f G_Ori_Traj;

    Vector3f G_TargetPos;
    Vector3f G_TargetVel;
    Vector3f G_TargetAcc;

    Vector3f G_TargetPrePos;
    Vector3f G_TargetPreVel;
    Vector3f G_TargetPreAcc;

    Vector3f G_TargetOri;
    Vector3f G_TargetAngVel;
    Vector3f G_TargetAngAcc;

    Vector3f G_TargetPreOri;
    Vector3f G_TargetPreAngVel;
    Vector3f G_TargetPreAngAcc;
    
    Vector3f tmp_G_TargetPos;
    Vector3f tmp_G_TargetVel;
    Vector3f tmp_G_TargetAcc;

    Vector3f tmp_G_TargetPrePos;
    Vector3f tmp_G_TargetPreVel;
    Vector3f tmp_G_TargetPreAcc;

    Vector3f tmp_G_TargetOri;
    Vector3f tmp_G_TargetAngVel;
    Vector3f tmp_G_TargetAngAcc;

    Vector3f tmp_G_TargetPreOri;
    Vector3f tmp_G_TargetPreAngVel;
    Vector3f tmp_G_TargetPreAngAcc;

    Vector6f X_Coeffs;
    Vector6f Y_Coeffs;
    Vector6f Z_Coeffs;
    Vector6f Roll_Coeffs;
    Vector6f Pitch_Coeffs;
    Vector6f Yaw_Coeffs;
}COMREF;

typedef struct Foot{
    Vector3f G_TargetPos;
    Vector3f G_TargetVel;
    Vector3f G_TargetAcc;

    Vector3f G_TargetPos_UP;
    Vector3f G_TargetVel_UP;
    Vector3f G_TargetAcc_UP;

    Vector3f G_TargetPrePos;
    Vector3f G_TargetPreVel;
    Vector3f G_TargetPreAcc;

    Vector3f G_TargetPos2;

    Vector3f G_TargetOri;
    Vector3f G_TargetAngVel;
    Vector3f G_TargetAngAcc;

    Vector3f G_TargetPreOri;
    Vector3f G_TargetPreAngVel;
    Vector3f G_TargetPreAngAcc;

    Vector3f G_InitPos;
    Vector3f G_InitVel;
    Vector3f G_InitAcc;

    Vector3f G_InitOri;
    Vector3f G_InitAngVel;
    Vector3f G_InitAngAcc;

    Vector3f G_Pos_Traj;
    Vector3f G_Ori_Traj;

    Vector6f X_Coeffs;
    Vector6f Y_Coeffs;
    Vector6f Z_Coeffs;
    Vector6f Z_Coeffs_UP;

    VectorXf ControlPointX       = VectorXf::Zero(9);
    VectorXf ControlPointY       = VectorXf::Zero(9);
    VectorXf ControlPointZ       = VectorXf::Zero(9);
}FOOT;

typedef struct Locomotion_Parameter{

    TIME time;
    GAIT gait;
    COMREF CoM_Ref;
    FOOT FL_Foot_Ref, FR_Foot_Ref, ML_Foot_Ref, MR_Foot_Ref, RL_Foot_Ref, RR_Foot_Ref; 
    JOYSTICK joystick;

    Vector4f contact_torque_threshold;

    Vector6f contact_foot;

    float imu_roll;
    float imu_pitch;
    Vector3f FL_Foot_ActPos_Local;
    Vector3f FR_Foot_ActPos_Local;
    Vector3f ML_Foot_ActPos_Local;
    Vector3f MR_Foot_ActPos_Local;
    Vector3f RL_Foot_ActPos_Local;
    Vector3f RR_Foot_ActPos_Local;
    bool snut_request;
    
}LOCOMOTION_PARAMETER;

// class Locomotion : public CRobot {
class Locomotion{
public:
    Locomotion();
    ~Locomotion();

    LOCOMOTION_PARAMETER parameter[1];

    void Locomotion_Trajectory_Generator(LOCOMOTION_PARAMETER& loco, Vector3f& CoM_RefPos, Vector3f& BASE_RefOri, Vector3f& FL_Foot_Ref, Vector3f& FR_Foot_Ref, Vector3f& ML_Foot_Ref, Vector3f& MR_Foot_Ref, Vector3f& RL_Foot_Ref, Vector3f& RR_Foot_Ref, Vector24f Current_Torque, Vector6f& Contact_Foot, Vector3f EstimatedAngle);
    void Locomotion_Trajectory_Generator(LOCOMOTION_PARAMETER& loco,
                                        //  Vector3f& CoM_RefPos, Vector3f& BASE_RefOri, Vector3f& FL_Foot_Ref, Vector3f& FR_Foot_Ref, Vector3f& ML_Foot_Ref, Vector3f& MR_Foot_Ref, Vector3f& RL_Foot_Ref, Vector3f& RR_Foot_Ref, Vector24f Current_Torque, Vector6f& Contact_Foot, Vector3f EstimatedAngle,
                                         const Vector3f FL_Foot_ActPos_Local,
                                         const Vector3f FR_Foot_ActPos_Local,
                                         const Vector3f ML_Foot_ActPos_Local,
                                         const Vector3f MR_Foot_ActPos_Local,
                                         const Vector3f RL_Foot_ActPos_Local,
                                         const Vector3f RR_Foot_ActPos_Local,
                                         const float imu_roll,
                                         const float imu_pitch,
                                         bool snut_request);
    void Locomotion_Timer(const float T_Stance, const float T_Swing, const float T_Period, float& T_Gait, float& T_Foot_Motion, bool& Stance_ON_Flag, bool& Swing_ON_Flag, bool& Swing_UP_Flag, bool& Swing_DOWN_Flag);
    void Locomotion_Initialize(void);
    void Gait_Selector(unsigned int Joystick_Press_Count, unsigned int& Gait_Type, bool& Gait_Check_Flag, float time_gait, float time_period);
    void Swing_Leg_Selector(unsigned int Gait_Type, unsigned int& Swing_Leg);
    void Contact_Foot_Checker(unsigned int Swing_Leg, unsigned int Gait_Phase, Vector6f& contact_foot);
    void Calc_CoM_Target_Pos(unsigned int Gait_Type, unsigned int Gait_Phase, const float T_Stance, const float T_Swing, Vector3f slope_angle);
    void Calc_Foot_Target_Pos(unsigned int Gait_Type, unsigned int Swing_Leg, const float T_Stance, const float T_Swing, const float Foot_Height);
    void CoM_Traj_Generator(Vector6f CoM_X_Coeffs, Vector6f CoM_Y_Coeffs, Vector6f CoM_Z_Coeffs, Vector6f CoM_Roll_Coeffs, Vector6f CoM_Pitch_Coeffs, Vector6f CoM_Yaw_Coeffs, unsigned int Gait_Phase, const float T_Stance, const float T_Swing, float time_gait);
    void Foot_Traj_Generator(unsigned int Gait_Phase, const float T_Stance, const float T_Swing, float time_gait, unsigned int Swing_Leg,
                                     Vector6f FL_Z_Coeffs, Vector6f FR_Z_Coeffs, Vector6f ML_Z_Coeffs, Vector6f MR_Z_Coeffs, Vector6f RL_Z_Coeffs, Vector6f RR_Z_Coeffs,
                                     Vector6f FL_Z_Coeffs_UP, Vector6f FR_Z_Coeffs_UP, Vector6f ML_Z_Coeffs_UP, Vector6f MR_Z_Coeffs_UP, Vector6f RL_Z_Coeffs_UP, Vector6f RR_Z_Coeffs_UP);
    Vector6f Contact_Estimate(Vector24f current_torque, Vector4f threshold, float swing_time, const float T_Swing);
    // Vector6f Contact_Estimate(const Vector24f& current_torque, const Vector4f& threshold, float swing_time, const float T_Swing, const unsigned int SwingLeg);

    // void SaveData();
    // void setPrintData(int DataLength, int SaveTime); 

    // void Calc_CoM_Target_Pos2(unsigned int Gait_Type, unsigned int Gait_Phase, const float T_Stance, const float T_Swing, 
    //                                  Vector3f* Target_Speed, Vector3f Speed_Scale_Tripod, Vector3f Speed_Scale_Ripple, Vector3f Speed_Scale_Wave, Vector3f Joystick_InputData,
    //                                  Vector3f* CoM_Target_Pos);



    //* 과기대에 추가해줄 내용
    // 스윙주기, 스탠스주기 등을 지정해주기 위함
    void Locomotion_Initialize(Vector3f FL_local_pos, Vector3f FR_local_pos, Vector3f ML_local_pos, Vector3f MR_local_pos, Vector3f RL_local_pos, Vector3f RR_local_pos,
                               float swing_time, float stance_time, float swing_height, float task_time);

    // 궤적을 그릴 때, 스윙궤적 그릴 시 다리가 "실제 위치" 부터 시작되길 바란다는 요청으로 추가한 내용
    void Calc_Foot_Target_Pos_for_snut(unsigned int Gait_Type, unsigned int Swing_Leg, const float T_Stance, const float T_Swing, const float Foot_Height,
                                float imu_roll, float imu_pitch,
                                Vector3f FL_Foot_ActPos,
                                Vector3f FR_Foot_ActPos,
                                Vector3f ML_Foot_ActPos,
                                Vector3f MR_Foot_ActPos,
                                Vector3f RL_Foot_ActPos,
                                Vector3f RR_Foot_ActPos);

    //* 헬퍼 함수들
    Matrix3f EulerZYX2RotMat(Vector3f RotationAngle);
    Vector3f RotMat2EulerZYX(Matrix3f RotMat);
    float cosWave(float Amp, float Period, float Time, float InitPos);
    float func_1_cos(float t, float init, float final, float T);
    float ramp_function(float t, float T);
    void Coefficient_5th_PolyNomial(float x_init, float dx_init, float d2x_init, float x_desired, float dx_desired, float d2x_desired, float time_period, Vector6f *Coeff_State);
    VectorXf Set_8thBezierControlPoint(Vector3f StartPoint, Vector3f EndPoint, int Axis, Matrix3f Rotation);
    float Function_8thBezierCurve(VectorXf ControlPoint, float Period, float Time);
    float Function_8thBezierCurve_dot(VectorXf ControlPoint, float Period, float Time);
    MatrixXf X_Rot(float q, int size);
    MatrixXf Y_Rot(float q, int size);
    MatrixXf Z_Rot(float q, int size);

private:
};

#endif /* LOCOMOTION_H */ 
