#pragma once

namespace ArmMotion {
    struct Joint6Dof_t {

        Joint6Dof_t() = default;

        Joint6Dof_t(double a1, double a2, double a3, double a4, double a5, double a6) 
            : theta{ a1, a2, a3, a4, a5, a6 } {}

        double theta[6];
    };
    
    struct Pose6Dof_t {

        Pose6Dof_t() = default;
        
        Pose6Dof_t(double X, double Y, double Z, double q1, double q2, double q3, double q4)
            : x(X), y(Y), z(Z), qx(q1), qy(q2), qz(q3), qw(q4),vaildR(false) {}

        double x{}, y{}, z{};
        // double roll{}, pitch{}, yaw{};
        double qx{}, qy{}, qz{}, qw{};
        double R[9]{};  // 列优先

        // 记录R变量的旋转矩阵是否有效
        bool vaildR{};
    };

    struct IKSolves_t {

        Joint6Dof_t solves[8];
        char solveFlag[8][3];

    };
}
