#ifndef MJUROBOTSTATE_HPP
#define MJUROBOTSTATE_HPP

#include "header.hpp"

enum Mode {
    INIT,
    READY,
    TEST,
    STAND,
    WALKING,
    SWIMMING_READY,
    SWIMMING
};

typedef struct
{
    struct
    {
        struct
        {
            double joint1 = 0;
            double joint2 = 0;
            double joint3 = 0;
            double joint4 = 0;

            double joint1_pre = 0;
            double joint2_pre = 0;
            double joint3_pre = 0;
            double joint4_pre = 0;

            double joint1_init = 0;
            double joint2_init = 0;
            double joint3_init = 0;
            double joint4_init = 0;

            double motor1 = 0;
            double motor2 = 0;
            double motor3 = 0;
            double motor4 = 0;

        }Pos, Vel, Acc, Tau;

        Eigen::Vector3d Tip_pos = Eigen::Vector3d::Zero();
        Eigen::Vector3d Tip_vel = Eigen::Vector3d::Zero();
        Eigen::Vector3d Tip_acc = Eigen::Vector3d::Zero();
        Eigen::Vector3d Tip_tau = Eigen::Vector3d::Zero();

        double x_init = 0;
        double y_init = 0;
        double z_init = 0;

        float CoM_x = 0;
        float CoM_y = 0;
        float CoM_z = 0;
        
        float CoM_roll  = 0;
        float CoM_pitch = 0;
        float CoM_yaw   = 0;
        
    }Sim, Ref;
    
}MjuState;

extern MjuState mjustate;

#endif
