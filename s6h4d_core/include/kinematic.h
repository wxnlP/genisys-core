#pragma once

#include <memory>

class KinematicSolver
{
public:
    struct Joint6Dof_t {

        Joint6Dof_t() = default;

        Joint6Dof_t(double a1, double a2, double a3, double a4, double a5, double a6) 
            : theta{ a1, a2, a3, a4, a5, a6 } {}

        double theta[6];

    };

    struct Pose6Dof_t {

        Pose6Dof_t() = default;
        
        Pose6Dof_t(double X, double Y, double Z, double a, double b, double c)
            : x(X), y(Y), z(Z), roll(a), pitch(b), yaw(c),vaildR(false) {}

        double x{}, y{}, z{};
        double roll{}, pitch{}, yaw{};
        double R[9]{};  // 列优先

        // 记录R变量的旋转矩阵是否有效
        bool vaildR{};

    };


    struct IKSolves_t {

        Joint6Dof_t solves[8];
        char solveFlag[8][3];

    };

    KinematicSolver(double L_BS, double D_BS, double L_AM, double L_FA, double D_EW, double L_WT);
    ~KinematicSolver();
    KinematicSolver(const KinematicSolver&) = delete;
    KinematicSolver& operator=(const KinematicSolver&) = delete;

    bool FK(const Joint6Dof_t &inputJoint, Pose6Dof_t &outputPose);
    bool IK(const Pose6Dof_t &inputPose, const Joint6Dof_t &lastJoint, IKSolves_t &outputSolves);

private:
    class Impl;
    std::unique_ptr<Impl> impl_;

    const double RAD_TO_DEG = 57.295777754771045;
    // 记录机械角度偏移
    double jointOffset_[6];
};
