#include "jacobian.h"
#include <Eigen/Dense>


class Jacobian6Dof::Impl 
{
public:
    // MDH params
    ArmMotion::MDHConfig_t mdhConfig_{};

    // 雅可比
    Eigen::Matrix<double, 6, 6> jacobian_;
    Eigen::Matrix<double, 6, 4> MDH_Matrix_;

    Impl(double L_BS, double D_BS, double L_AM, double D_EW, double L_FA, double L_WT)
        : mdhConfig_(ArmMotion::MDHConfig_t{ L_BS, D_BS, L_AM, D_EW, L_FA, L_WT })
    {
        MDH_Matrix_ <<
            0.0000          , 0.00                  , mdhConfig_.L_BASE    , 0.0 ,
            -(double)M_PI_2 , mdhConfig_.D_BASE     , 0.00                 , 0.0 ,
            0.0000          , mdhConfig_.L_UPPERARM , 0.00                 , 0.0 ,
            -(double)M_PI_2 , mdhConfig_.D_ELBOW    , mdhConfig_.L_FOREARM , 0.0 ,
            (double)M_PI_2  , 0.00                  , 0.00                 , 0.0 ,
            -(double)M_PI_2 , 0.00                  , mdhConfig_.L_WRIST   , 0.0 ;
    }
};

Jacobian6Dof::Jacobian6Dof(double L_BS, double D_BS, double L_AM, double L_FA, double D_EW, double L_WT)
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


void Jacobian6Dof::Update(const ArmMotion::Joint6Dof_t &inputJoint)
{
    // 齐次矩阵
    Eigen::Matrix4d T_02, T_03, T_04, T_05, T_06;
    Eigen::Matrix4d T[6];
    // 雅可比构造参数
    Eigen::Vector3d z[6];
    Eigen::Vector3d p[6];
    Eigen::Vector3d pe;
    double theta[6];

    for (int i = 0; i < 6; i++) {
        
        theta[i] = inputJoint.theta[i] + jointOffset_[i];

        double st = std::sin(theta[i]);
        double ct = std::cos(theta[i]);
        double sa = std::sin(impl_->MDH_Matrix_(i, 0));
        double ca = std::cos(impl_->MDH_Matrix_(i, 0));
        double a = impl_->MDH_Matrix_(i, 1);
        double d = impl_->MDH_Matrix_(i, 2);

        T[i] << ct,    -st,   0.0, a,
                st*ca, ct*ca, -sa, -sa*d,
                st*sa, ct*sa, ca,  ca*d,
                0.0,   0.0,   0.0, 1.0;
    }

    T_02 = T[0] * T[1];
    T_03 = T_02 * T[2];
    T_04 = T_03 * T[3];
    T_05 = T_04 * T[4];
    T_06 = T_05 * T[5];

    p[0] = T[0].block<3, 1>(0, 3);
    p[1] = T_02.block<3, 1>(0, 3);
    p[2] = T_03.block<3, 1>(0, 3);
    p[3] = T_04.block<3, 1>(0, 3);
    p[4] = T_05.block<3, 1>(0, 3);
    p[5] = T_06.block<3, 1>(0, 3);
    pe = T_06.block<3, 1>(0, 3);
    z[0] = T[0].block<3, 1>(0, 2);
    z[1] = T_02.block<3, 1>(0, 2);
    z[2] = T_03.block<3, 1>(0, 2);
    z[3] = T_04.block<3, 1>(0, 2);
    z[4] = T_05.block<3, 1>(0, 2);
    z[5] = T_06.block<3, 1>(0, 2);

    // 构建雅可比矩阵
    for (int i = 0; i < 6; i++) {
        // 线速度
        impl_->jacobian_.block<3, 1>(0, i) = z[i].cross(pe-p[i]);
        // 角速度
        impl_->jacobian_.block<3, 1>(3, i) = z[i];
    }
}

bool Jacobian6Dof::ForwardVelocity(const JointVel &inputVel, CartesianVel &outputVel)
{
    // 映射只读
    Eigen::Map<const Eigen::Vector<double, 6>> jointVel(&inputVel.qv[0]);
    Eigen::Vector<double, 6> cartesianVel = impl_->jacobian_ * jointVel;

    // 映射可写
    Eigen::Map<Eigen::Vector<double, 6>> outMap(&outputVel.v[0]);
    outMap = cartesianVel;

    return true;
}

bool Jacobian6Dof::InverseVelocity(const CartesianVel &inputVel, JointVel &outputVel, double damping)
{
    Eigen::Map<const Eigen::Vector<double, 6>> vTarget(&inputVel.v[0]);
    // 求雅可比伪逆
    Eigen::Matrix<double, 6, 6> J = impl_->jacobian_;
    Eigen::Matrix<double, 6, 6> JT = J.transpose();
    Eigen::Matrix<double, 6, 6> JJT = J * JT;

    for (int i = 0; i < 6; ++i) {
        JJT(i, i) += (damping * damping);
    }

    // 使用LLDT计算代替求逆
    Eigen::Vector<double, 6> x = JJT.ldlt().solve(vTarget);
    Eigen::Vector<double, 6> dqResult = JT * x;

    Eigen::Map<Eigen::Vector<double, 6>> outMap(&outputVel.qv[0]);
    outMap = dqResult;
    return true;
}

Jacobian6Dof::~Jacobian6Dof() = default; 