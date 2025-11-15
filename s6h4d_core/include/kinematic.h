#pragma once

class KinematicSolver
{
public:
    typedef struct Joint6Dof_t {

        Joint6Dof_t() = default;

        Joint6Dof_t(float a1, float a2, float a3, float a4, float a5, float a6) 
            : theta{ a1, a2, a3, a4, a5, a6 } {}

        float theta[6];

    } Joint6Dof_t;

    typedef struct Pose6Dof_t {

        Pose6Dof_t() = default;
        
        Pose6Dof_t(float X, float Y, float Z, float a, float b, float c)
            : x(X), y(Y), z(Z), roll(a), pitch(b), yaw(c),vaildR(false) {}

        float x{}, y{}, z{};
        float roll{}, pitch{}, yaw{};
        float R[9]{};

        // 记录R变量的旋转矩阵是否有效
        bool vaildR{};

    } Pose6Dof_t;


    typedef struct IKSolves_t {

        Joint6Dof_t solves[8];
        char solveFlag[8][3];

    } IKSolves_t;

    KinematicSolver(float L_BS, float D_BS, float L_AM, float L_FA, float D_EW, float L_WT);

    bool FK(const Joint6Dof_t &inputJoint, Pose6Dof_t &outputPose);
    bool IK(const Pose6Dof_t &inputPose, const Joint6Dof_t &lastJoint, IKSolves_t &outputSolves);

private:

    // MDH params
    typedef struct {
        float L_BASE;
        float D_BASE;
        float L_UPPERARM;
        float D_ELBOW;
        float L_FOREARM;
        float L_WRIST;
    } MDHConfig_t;

    MDHConfig_t mdhConfig_{};

    const float RAD_TO_DEG = 57.295777754771045;
    float MDH_Matrix_[6][4]{0};

    float L0_Base_[3]{0};
    float L2_Arm_[3]{0};
    float L3_Elbow_[3]{0};
    float L6_Wrist_[3]{0};
};
