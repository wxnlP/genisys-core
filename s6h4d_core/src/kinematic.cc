#include "kinematic.h"
#include <cmath>
#include <cstring>
#include <Eigen/Dense>

static void Rotation2Euler(const double *R, double *euler)
{
    Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::ColMajor>> m(R);
    
    double roll, pitch, yaw;

    double s_beta = -m(2, 0);
    double c_beta = sqrt(m(0, 0) * m(0, 0) + m(1, 0) * m(1, 0));

    if (c_beta > 0.000001) {
        roll  = std::atan2(m(2, 1), m(2, 2));
        pitch = std::atan2(s_beta, c_beta);
        yaw = std::atan2(m(1, 0), m(0, 0));
    } else {
        yaw = 0.0;
        pitch = (s_beta > 0) ? (double)M_PI_2 : -(double)M_PI_2;
        roll = std::atan2(-m(0, 1), m(1, 1));
    }

    euler[0] = roll; // roll
    euler[1] = pitch; // pitch
    euler[2] = yaw; // yaw
}

static void Euler2Rotation(const double *euler, double *R)
{
    Eigen::AngleAxisd roll(euler[0], Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitch(euler[1], Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yaw(euler[2], Eigen::Vector3d::UnitZ());
    
    Eigen::Matrix3d m = (yaw * pitch * roll).toRotationMatrix();
    
    // 修复：在构造Eigen::Map对象时直接传入指针R，然后赋值
    Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::ColMajor>>(R, 3, 3) = m;
}

class KinematicSolver::Impl 
{
public:
    // MDH params
    struct MDHConfig_t{
        double L_BASE;
        double D_BASE;
        double L_UPPERARM;
        double D_ELBOW;
        double L_FOREARM;
        double L_WRIST;
    };

    MDHConfig_t mdhConfig_{};

    Eigen::Matrix<double, 6, 4> MDH_Matrix_;

    Eigen::Vector3d L0_Base_;
    Eigen::Vector3d L2_Arm_;
    Eigen::Vector3d L3_Elbow_;
    Eigen::Vector3d L6_Wrist_;
    Impl(double L_BS, double D_BS, double L_AM, double D_EW, double L_FA, double L_WT)
        : mdhConfig_(MDHConfig_t{ L_BS, D_BS, L_AM, D_EW, L_FA, L_WT })
    {
        MDH_Matrix_ <<
            0.0000          , 0.00                  , mdhConfig_.L_BASE    , 0.0 ,
            -(double)M_PI_2 , mdhConfig_.D_BASE     , 0.00                 , 0.0 ,
            0.0000          , mdhConfig_.L_UPPERARM , 0.00                 , 0.0 ,
            -(double)M_PI_2 , mdhConfig_.D_ELBOW    , mdhConfig_.L_FOREARM , 0.0 ,
            (double)M_PI_2  , 0.00                  , 0.00                 , 0.0 ,
            -(double)M_PI_2 , 0.00                  , mdhConfig_.L_WRIST   , 0.0 ;

        L0_Base_   = Eigen::Vector3d(mdhConfig_.D_BASE, 0.0, mdhConfig_.L_BASE);
        L2_Arm_    = Eigen::Vector3d(mdhConfig_.L_UPPERARM, 0.0, 0.0);
        L3_Elbow_  = Eigen::Vector3d(mdhConfig_.D_ELBOW, mdhConfig_.L_FOREARM, 0.0);
        L6_Wrist_  = Eigen::Vector3d(0.0, 0.0, -mdhConfig_.L_WRIST);
    }
};

KinematicSolver::KinematicSolver(double L_BS, double D_BS, double L_AM, double D_EW, double L_FA, double L_WT)
    : impl_(std::make_unique<Impl>(L_BS, D_BS, L_AM, D_EW, L_FA, L_WT))
{

    jointOffset_[0] = 0.0;
    jointOffset_[1] = -(double)M_PI_2; 
    // jointOffset_[1] = 0.0f;
    jointOffset_[2] = 0.0;
    jointOffset_[3] = 0.0;
    jointOffset_[4] = 0.0;
    jointOffset_[5] = 0.0;
}


bool KinematicSolver::FK(const ArmMotion::Joint6Dof_t &inputJoint, 
                         ArmMotion::Pose6Dof_t &outputPose)
{
    double theta_in[6];
    double theta[6];
    Eigen::Matrix3d R[6];
    Eigen::Vector3d P_06_pos;
    Eigen::Matrix3d R_02, R_03, R_04, R_05, R_06;
    Eigen::Vector3d L0_Base;
    Eigen::Vector3d L0_Arm;
    Eigen::Vector3d L0_Elbow;
    Eigen::Vector3d L0_Wrist;

    // deg to rad
    for (int i = 0; i < 6; i++) {
        theta_in[i] = inputJoint.theta[i];
    }
    
    for (int i = 0; i < 6; i++) {
        
        theta[i] = theta_in[i] + jointOffset_[i];

        double st = std::sin(theta[i]);
        double ct = std::cos(theta[i]);
        double sa = std::sin(impl_->MDH_Matrix_(i, 0));
        double ca = std::cos(impl_->MDH_Matrix_(i, 0));

        R[i] << ct,    -st,   0.0,
                st*ca, ct*ca, -sa,
                st*sa, ct*sa, ca;
    }
    
    R_02 = R[0] * R[1];
    R_03 = R_02 * R[2];
    R_04 = R_03 * R[3];
    R_05 = R_04 * R[4];
    R_06 = R_05 * R[5];

    L0_Base = R[0] * impl_->L0_Base_;
    L0_Arm  = R_02 * impl_->L2_Arm_;
    L0_Elbow = R_03 * impl_->L3_Elbow_;
    L0_Wrist = R_06 * impl_->L6_Wrist_;

    // position
    P_06_pos.head<3>() = L0_Base + L0_Arm + L0_Elbow - L0_Wrist;

    // posture
    // Rotation2Euler(R_06.data(), P_06_euler.data());
    Eigen::Quaterniond P_06_quat(R_06);
    P_06_quat.normalize();

    outputPose.x = P_06_pos[0];
    outputPose.y = P_06_pos[1];
    outputPose.z = P_06_pos[2];
    // outputPose.roll = P_06_euler[0];
    // outputPose.pitch = P_06_euler[1];
    // outputPose.yaw = P_06_euler[2];
    outputPose.quat = ArmMotion::Quaternion_t(
        P_06_quat.w(), P_06_quat.x(), P_06_quat.y(), P_06_quat.z());

    memcpy(outputPose.R, R_06.data(),  9 * sizeof(double));

    return true;
}


bool KinematicSolver::IK(const ArmMotion::Pose6Dof_t &inputPose, 
                         const ArmMotion::Joint6Dof_t &lastJoint, 
                         ArmMotion::IKSolves_t &outputSolves)
{
    double theta_1[2]{};
    double theta_23[2][2]{};
    double theta_456[2][3]{};

    Eigen::Vector3d P_06_pos;
    Eigen::Quaterniond P_06_quat;
    Eigen::Vector3d P_0w;
    Eigen::Vector3d L_2w;
    Eigen::Matrix3d R_10;
    Eigen::Matrix3d R_31;
    Eigen::Matrix3d R_30;
    Eigen::Matrix3d R_36;
    Eigen::Matrix3d R_06;

    Eigen::Vector3d L0_Wrist;

    P_06_pos[0] = inputPose.x;
    P_06_pos[1] = inputPose.y;
    P_06_pos[2] = inputPose.z;

    if (inputPose.vaildR) {
        memcpy(R_06.data(), inputPose.R, 9 * sizeof(double));
    } else {
        // deg to rad
        P_06_quat = Eigen::Quaterniond(inputPose.quat.w, 
                                       inputPose.quat.x, 
                                       inputPose.quat.y, 
                                       inputPose.quat.z);
        // P_06_quat.x() = inputPose.qx;
        // P_06_quat.y() = inputPose.qy;
        // P_06_quat.z() = inputPose.qz;
        // P_06_quat.w() = inputPose.qw;

        // quaternion to rotation
        P_06_quat.normalize();
        R_06 = P_06_quat.toRotationMatrix();
        // Euler2Rotation(P_06_euler.data(), R_06.data());
    }

    for (int i = 0; i < 2; i++) {
        theta_1[i] = lastJoint.theta[0] + jointOffset_[0];
        theta_23[i][0] = lastJoint.theta[1] + jointOffset_[1];
        theta_23[i][1] = lastJoint.theta[2] + jointOffset_[2];
        theta_456[i][0] = lastJoint.theta[3] + jointOffset_[3];
        theta_456[i][1] = lastJoint.theta[4] + jointOffset_[4];
        theta_456[i][2] = lastJoint.theta[5] + jointOffset_[5];
    }

    L0_Wrist = R_06 * impl_->L6_Wrist_;

    P_0w.head<3>() = P_06_pos + L0_Wrist;

    /* ---------------------------------J1-------------------------------------- */
    if ((P_0w[0] * P_0w[0] + P_0w[1] * P_0w[1]) <= 0.000001) {
        theta_1[0] = lastJoint.theta[0];
        theta_1[1] = theta_1[0];
        for (int i = 0; i < 4; i++) {
            outputSolves.solveFlag[0 + i][0] = -1;
            outputSolves.solveFlag[4 + i][0] = -1;
        }
    } else {
        theta_1[0] = std::atan2(P_0w[1], P_0w[0]);
        theta_1[1] = std::atan2(-P_0w[1], -P_0w[0]);
        for (int i = 0; i < 4; i++) {
            outputSolves.solveFlag[0 + i][0] = 1;
            outputSolves.solveFlag[4 + i][0] = 1;
        }
    }

    /* ---------------------------------J23------------------------------------- */
    for (int ind_arm = 0; ind_arm < 2; ind_arm++) {
        double ct1 = std::cos(theta_1[ind_arm]);
        double st1 = std::sin(theta_1[ind_arm]);

        R_10 << ct1, st1, 0.0,
                -st1, ct1, 0.0,
                0.0,   0.0,   1.0;

        Eigen::Vector3d P_0w_r;

        // rotation
        P_0w_r = R_10 * P_0w;

        L_2w = P_0w_r - impl_->L0_Base_;

        double c = sqrt(L_2w[0]*L_2w[0] + L_2w[2]*L_2w[2]);
        double b = sqrt(
            impl_->mdhConfig_.D_ELBOW * impl_->mdhConfig_.D_ELBOW + impl_->mdhConfig_.L_FOREARM * impl_->mdhConfig_.L_FOREARM);
        double a = impl_->mdhConfig_.L_UPPERARM;
        
        if (fabs(a + b - c) <= 0.000001) {
            theta_23[0][0] = std::atan2(L_2w[2], L_2w[0]);
            theta_23[1][0] = theta_23[0][0];

            theta_23[0][1] = 0.0;
            theta_23[1][1] = 0.0;

            if (c > a + b) {
                for (int i = 0; i < 2; i++) {
                    outputSolves.solveFlag[0 + i + 4 * ind_arm][1] = 0;
                    outputSolves.solveFlag[2 + i + 4 * ind_arm][1] = 0;
                }
            } else {
                for (int i = 0; i < 2; i++) {
                    outputSolves.solveFlag[0 + i + 4 * ind_arm][1] = 1;
                    outputSolves.solveFlag[2 + i + 4 * ind_arm][1] = 1;
                }
            }
        } else if (fabs(c - fabs(a - b) <= 0.000001)) {
            theta_23[0][0] = std::atan2(L_2w[2], L_2w[0]);
            theta_23[1][0] = theta_23[0][0];

            if (ind_arm == 0) {
                theta_23[0][1] = (double)M_PI;
                theta_23[1][1] = -(double)M_PI;
            } else {
                theta_23[0][1] = -(double)M_PI;
                theta_23[1][1] = (double)M_PI;
            }

            if (c < fabs(a - b)) {
                for (int i = 0; i < 2; i++) {
                    outputSolves.solveFlag[0 + i + 4 * ind_arm][1] = 0;
                    outputSolves.solveFlag[2 + i + 4 * ind_arm][1] = 0;
                }
            } else {
                for (int i = 0; i < 2; i++) {
                    outputSolves.solveFlag[0 + i + 4 * ind_arm][1] = 1;
                    outputSolves.solveFlag[2 + i + 4 * ind_arm][1] = 1;
                }
            }
        } else {
            double beta2, beta3;
            double q3 = (a*a + b*b - c*c) / (2*a*b);

            if (q3 >= 1.0) {
                beta3 = 0.0;
            } else if (q3 <= -1.0) {
                beta3 = (double)M_PI;
            } else {
                beta3 = std::acos(q3);
            }

            double q2 = (a*a + c*c - b*b) / (2*a*c);
            if (q2 >= 1.0) {
                beta2 = 0.0;
            } else if (q2 <= -1.0) {
                beta2 = (double)M_PI;
            } else {
                beta2 = std::acos(q2);
            }

            double alpha2 = std::atan2(L_2w[2], L_2w[0]);
            double alpha3 = std::atan2(impl_->mdhConfig_.L_FOREARM, impl_->mdhConfig_.D_ELBOW);

            if (ind_arm == 0) {
                theta_23[0][0] = alpha2 + beta2;
                theta_23[0][1] = -((double)M_PI - alpha3 - beta3);
                theta_23[1][0] = -(alpha2 + beta2);
                theta_23[1][1] = (double)M_PI - alpha3 - beta3;
            } else {
                theta_23[0][0] = -(alpha2 + beta2);
                theta_23[0][1] = (double)M_PI - alpha3 - beta3;
                theta_23[1][0] = alpha2 + beta2;
                theta_23[1][1] = -((double)M_PI - alpha3 - beta3);
            }
            for (int i = 0; i < 2; i++) {
                outputSolves.solveFlag[0 + i + 4 * ind_arm][1] = 1;
                outputSolves.solveFlag[2 + i + 4 * ind_arm][1] = 1;
            }
        }
        for (int ind_elbow = 0; ind_elbow < 2; ind_elbow++) {
            double c2 = std::cos(theta_23[ind_elbow][0]);
            double s2 = std::sin(theta_23[ind_elbow][0]);
            double c3 = std::cos(theta_23[ind_elbow][1]);
            double s3 = std::sin(theta_23[ind_elbow][1]);

            R_31 << (c2 * c3 - s2 * s3), 0.0, -(c2 * s3 + s2 * c3),
                    -(c2 * s3 + s2 * c3), 0.0, -(c2 * c3 - s2 * s3),
                    0.0, 1.0, 0.0;

            R_30 = R_31 * R_10;
            R_36 = R_30 * R_06;

            double cos5;
            if (R_36(1, 2) > 1.0 - 0.00001) {
                cos5 = 1.0;
                theta_456[0][1] = 0.0;
                theta_456[1][1] = 0.0;
            } else if (R_36(1, 2) < -1.0 + 0.00001) {
                cos5 = -1.0;
                if (ind_arm == 0) {
                    theta_456[0][1] = (double)M_PI;
                    theta_456[1][1] = -(double)M_PI;
                } else {
                    theta_456[0][1] = -(double)M_PI;
                    theta_456[1][1] = (double)M_PI;
                }
            } else {
                cos5 = R_36(1, 2);
                if (ind_arm == 0) {
                    theta_456[0][1] = std::acos(cos5);
                    theta_456[1][1] = -std::acos(cos5);
                } else {
                    theta_456[0][1] = -std::acos(cos5);
                    theta_456[1][1] = std::acos(cos5);
                }
            }
            if (cos5 == 1.0 || cos5 == -1.0) {
                if (ind_arm == 0) {
                    theta_456[0][0] = lastJoint.theta[3] + jointOffset_[3];
                    double c4 = std::cos(lastJoint.theta[3] + jointOffset_[3]);
                    double s4 = std::sin(lastJoint.theta[3] + jointOffset_[3]);
                    theta_456[0][2] = std::atan2(-s4*R_36(0, 0) - c4*R_36(2, 0), -s4*R_36(0, 1) - c4*R_36(2, 1));

                    theta_456[1][2] = lastJoint.theta[5] + jointOffset_[5];
                    double c6 = std::cos(lastJoint.theta[5] + jointOffset_[5]);
                    double s6 = std::sin(lastJoint.theta[5] + jointOffset_[5]);
                    theta_456[1][0] = std::atan2(-s6*R_36(0, 0) - c6*R_36(0, 1), -s6*R_36(2, 0) - c6*R_36(2, 1));
                } else {
                    theta_456[0][2] = lastJoint.theta[5] + jointOffset_[5];
                    double c6 = std::cos(lastJoint.theta[5] + jointOffset_[5]);
                    double s6 = std::sin(lastJoint.theta[5] + jointOffset_[5]);
                    theta_456[0][0] = std::atan2(-s6*R_36(0, 0) - c6*R_36(0, 1), -s6*R_36(2, 0) - c6*R_36(2, 1));

                    theta_456[1][0] = lastJoint.theta[3] + jointOffset_[3];
                    double c4 = std::cos(lastJoint.theta[3] + jointOffset_[3]);
                    double s4 = std::sin(lastJoint.theta[3] + jointOffset_[3]);
                    theta_456[1][2] = std::atan2(-s4*R_36(0, 0) - c4*R_36(2, 0), -s4*R_36(0, 1) - c4*R_36(2, 1));

                }

                outputSolves.solveFlag[4 * ind_arm + 2 * ind_elbow + 0][2] = -1;
                outputSolves.solveFlag[4 * ind_arm + 2 * ind_elbow + 1][2] = -1;
            } else {
                if (ind_arm == 0) {
                    theta_456[0][0] = std::atan2(R_36(2, 2), -R_36(0, 2));
                    theta_456[1][0] = std::atan2(-R_36(2, 2), R_36(0, 2));
                    theta_456[0][2] = std::atan2(-R_36(1, 1), R_36(1, 0));
                    theta_456[1][2] = std::atan2(R_36(1, 1), -R_36(1, 0));
                } else {
                    theta_456[1][0] = std::atan2(R_36(2, 2), -R_36(0, 2));
                    theta_456[0][0] = std::atan2(-R_36(2, 2), R_36(0, 2));
                    theta_456[1][2] = std::atan2(-R_36(1, 1), R_36(1, 0));
                    theta_456[0][2] = std::atan2(R_36(1, 1), -R_36(1, 0));
                }
                outputSolves.solveFlag[4 * ind_arm + 2 * ind_elbow + 0][2] = 1;
                outputSolves.solveFlag[4 * ind_arm + 2 * ind_elbow + 1][2] = 1;
            }

            for (int ind_wrist = 0; ind_wrist < 2; ind_wrist++) {
                double theta_1_ = theta_1[ind_arm] - jointOffset_[0];
                // while (theta_1_ > (double)M_PI) {
                //     theta_1_ -= 2.0 * (double)M_PI;
                // }
                        
                // while (theta_1_ <= -(double)M_PI) {
                //     theta_1_ += 2.0 * (double)M_PI;
                // }
                outputSolves.solves[ind_arm * 4 + ind_elbow * 2 + ind_wrist].theta[0] = theta_1_;

                for(int i = 0; i < 2; i++) {
                    double theta_23_ = theta_23[ind_elbow][i] - jointOffset_[1 + i];
                    // while (theta_23_ > (double)M_PI) {
                    //     theta_23_ -= 2.0 * (double)M_PI;
                    // }
                    // while (theta_23_ <= -(double)M_PI) {
                    //     theta_23_ += 2.0 * (double)M_PI;
                    // }
                    outputSolves.solves[ind_arm * 4 + ind_elbow * 2 + ind_wrist].theta[1+i] = theta_23_;

                }

                for(int i = 0; i < 3; i++) {
                    double theta_456_ = theta_456[ind_wrist][i] - jointOffset_[2 + i];
                    // while (theta_456_ > (double)M_PI) {
                    //     theta_456_ -= 2.0 * (double)M_PI;
                    // }
                    // while (theta_456_ <= -(double)M_PI) {
                    //     theta_456_ += 2.0 * (double)M_PI;
                    // }
                    outputSolves.solves[ind_arm * 4 + ind_elbow * 2 + ind_wrist].theta[3+i] = theta_456_;
                }
            }
        }
    }

    return true;
}

KinematicSolver::~KinematicSolver() = default; 