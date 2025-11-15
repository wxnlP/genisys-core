#include "kinematic.h"
#include <cmath>
#include <cstring>
#include <iostream>


#ifdef ARM_MATH
inline float cosf(float x)
{
    return arm_cos_f32(x);
}

inline float sinf(float x)
{
    return arm_sin_f32(x);
}
// #else
// inline float cosf(float x)
// {
//     return std::cos(x);
// }

// inline float sinf(float x)
// {
//     return std::sin(x);
// }
#endif


static void Rotation2Euler(const float *R, float *euler)
{
    float alpha{}, beta{}, gama{};

    if (fabs(R[6]) >= 1.0f - 0.00001f) {
        if (R[6] < 0) {
            beta = (float)M_PI_2;
            alpha = 0.0f;
            gama = atan2f(R[1], R[4]);
        } else {
            beta = -(float)M_PI_2;
            alpha = 0.0f;
            gama = -atan2f(R[1], R[4]);
        }
    } else {
        beta = atan2f(-R[6], sqrtf(R[0] * R[0] + R[3] * R[3]));
        float temp = cosf(beta);
        alpha = atan2f(R[3] / temp, R[0] / temp);
        gama = atan2f(R[7] / temp, R[8] / temp);
    }
    
    euler[0] = gama;
    euler[1] = beta;
    euler[2] = alpha;
}

static void Euler2Rotation(const float *euler, float *R)
{
    float sa = sinf(euler[2]);
    float ca = cosf(euler[2]);
    float sb = sinf(euler[1]);
    float cb = cosf(euler[1]);
    float sg = sinf(euler[0]);
    float cg = cosf(euler[0]);

    R[0] = ca * cb;
    R[1] = ca * sb * sg - sa * cg;
    R[2] = ca * sb * cg + sa * sg;
    R[3] = sa * cb;
    R[4] = sa * sb * sg + ca * cg;
    R[5] = sa * sb * cg - ca * sg;
    R[6] = -sb;
    R[7] = cb * sg;
    R[8] = cb * cg;
}

/**
 * @brief 两个矩阵相乘
 * @note 记住：
 *          1. 矩阵1的列=矩阵2的行 = n1
 *          2. 结果矩阵的行=矩阵1的行 = m1
 *          3. 结果矩阵的列=矩阵2的列 = n2
 * @param matrix1  第一个矩阵 
 * @param matrix2  第二个矩阵
 * @param matrixOut 结果矩阵 
 * @param m1        结果矩阵的行数 / 矩阵1的行数
 * @param n1        矩阵1的列数 / 矩阵2的行数
 * @param n2        结果矩阵的列数 / 矩阵2的列数
 */
static void MatrixMultiply(const float *matrix1, const float *matrix2, float *outputMatrix, 
    const int m1, const int n1, const int n2)
{
    int i, j, k;
    float temp;

    for (i = 0; i < m1; i++) {
        for (j = 0; j < n2; j++) {
            temp = 0.0f;
            for (k = 0; k < n1; k++) {
                temp += matrix1[i * n1 + k] * matrix2[j + k * n2];
            }
            outputMatrix[i * n2 + j] = temp;
        }
    }
}

template <typename T>
void print_matrix(const T* data, int rows, int cols) {
    // 检查指针是否为空
    if (data == nullptr) {
        std::printf("Error: Matrix data is null.\n");
        return;
    }

    // 检查行数或列数是否合法
    if (rows <= 0 || cols <= 0) {
        std::printf("Error: Invalid matrix dimensions (rows: %d, cols: %d).\n", rows, cols);
        return;
    }

    // 打印矩阵
    std::printf("Matrix (%d x %d):\n", rows, cols);
    for (int i = 0; i < rows; ++i) { // 遍历行
        for (int j = 0; j < cols; ++j) { // 遍历列
            // 计算当前元素在一维数组中的索引: index = i * cols + j
            int index = i * cols + j;

            // 打印元素
            // 注意：printf的格式化字符串需要根据T的实际类型进行调整。
            // 假设 T 为 int 或 float/double，并使用合适的宽度进行对齐。
            if constexpr (std::is_integral_v<T>) {
                std::printf("%6d ", data[index]); // 整数，使用6个字符宽度对齐
            } else if constexpr (std::is_floating_point_v<T>) {
                std::printf("%6.2f ", data[index]); // 浮点数，使用6个字符宽度，小数点后保留2位
            } else {
                // 如果是其他类型，您可能需要专门的打印逻辑，这里作为默认处理
                std::printf("? "); 
            }
        }
        // 每行结束时换行
        std::printf("\n");
    }
}

KinematicSolver::KinematicSolver(float L_BS, float D_BS, float L_AM, float D_EW, float L_FA, float L_WT)
    : mdhConfig_(MDHConfig_t{ L_BS, D_BS, L_AM, D_EW, L_FA, L_WT })
{

    // std::printf("%f\n", mdhConfig_.L_BASE);
    // std::printf("%f\n", mdhConfig_.D_BASE);
    // std::printf("%f\n", mdhConfig_.L_UPPERARM);
    // std::printf("%f\n", mdhConfig_.D_ELBOW);
    // std::printf("%f\n", mdhConfig_.L_FOREARM);
    // std::printf("%f\n", mdhConfig_.L_WRIST);

    float tempMdhMatrix[6][4] = {
        {  0.0000f        , 0.00f                 , mdhConfig_.L_BASE   , 0.0f },
        { -(float)M_PI_2 , mdhConfig_.D_BASE     , 0.00f                , 0.0f },
        {  0.0000f       , mdhConfig_.L_UPPERARM , 0.00f                , 0.0f },
        { -(float)M_PI_2 , mdhConfig_.D_ELBOW    , mdhConfig_.L_FOREARM , 0.0f },
        {  (float)M_PI_2 , 0.00f                 , 0.00f                , 0.0f },
        { -(float)M_PI_2 , 0.00f                 , mdhConfig_.L_WRIST   , 0.0f }
    };

    memcpy(MDH_Matrix_, tempMdhMatrix, sizeof(tempMdhMatrix));

    float tempL0_Base[3] = { mdhConfig_.D_BASE, 0.0f, mdhConfig_.L_BASE };
    float tempL2_Arm[3] = { mdhConfig_.L_UPPERARM, 0.0f, 0.0f };
    float tempL3_Elbow[3] = { mdhConfig_.D_ELBOW, mdhConfig_.L_FOREARM, 0.0f };
    float tempL6_Wrist[3] = { 0.0f, 0.0f, -mdhConfig_.L_WRIST };

    memcpy(L0_Base_, tempL0_Base, sizeof(tempL0_Base));
    memcpy(L2_Arm_, tempL2_Arm, sizeof(tempL2_Arm));
    memcpy(L3_Elbow_, tempL3_Elbow, sizeof(tempL3_Elbow));
    memcpy(L6_Wrist_, tempL6_Wrist, sizeof(tempL6_Wrist));
    
}


bool KinematicSolver::FK(const Joint6Dof_t &inputJoint, Pose6Dof_t &outputPose)
{
    float theta_in[6]{};
    float theta[6]{};
    float R[6][9]{};
    float P_06[6]{};
    float R_02[9]{};
    float R_03[9]{};
    float R_04[9]{};
    float R_05[9]{};
    float R_06[9]{};
    float L0_Base[3]{};
    float L0_Arm[3]{};
    float L0_Elbow[3]{};
    float L0_Wrist[3]{};

    // deg to rad
    for (int i = 0; i < 6; i++) {
        theta_in[i] = inputJoint.theta[i] / RAD_TO_DEG;
    }
    
    for (int i = 0; i < 6; i++) {
        
        theta[i] = theta_in[i] + MDH_Matrix_[i][3];

        // std::printf("theta[%d]:%f\n", i, theta[i]);

        float st = sinf(theta[i]);
        float ct = cosf(theta[i]);
        float sa = sinf(MDH_Matrix_[i][0]);
        float ca = cosf(MDH_Matrix_[i][0]);

        R[i][0] = ct;
        R[i][1] = -st;
        R[i][2] = 0.0f;
        R[i][3] = st * ca;
        R[i][4] = ct * ca;
        R[i][5] = -sa;
        R[i][6] = st * sa;
        R[i][7] = ct * sa;
        R[i][8] = ca;
    }
    
    MatrixMultiply(R[0], R[1], R_02, 3, 3, 3);
    MatrixMultiply(R_02, R[2], R_03, 3, 3, 3);
    MatrixMultiply(R_03, R[3], R_04, 3, 3, 3);
    MatrixMultiply(R_04, R[4], R_05, 3, 3, 3);
    MatrixMultiply(R_05, R[5], R_06, 3, 3, 3);

    // print_matrix(R_06, 3, 3);

    // for (int i = 0; i < 3; i++) {
    //     std::printf("%f\t", L3_Elbow_[i]);
    // }
    // std::printf("\n");

    MatrixMultiply(R[0], L0_Base_, L0_Base, 3, 3, 1);
    MatrixMultiply(R_02, L2_Arm_, L0_Arm, 3, 3, 1);
    MatrixMultiply(R_03, L3_Elbow_, L0_Elbow, 3, 3, 1);
    MatrixMultiply(R_06, L6_Wrist_, L0_Wrist, 3, 3, 1);

    // print_matrix(R_03, 3, 3);

    
    // for (int i = 0; i < 3; i++) {
    //     std::printf("%f\t", L0_Base[i]);
    // }
    // std::printf("\n");

    // for (int i = 0; i < 3; i++) {
    //     std::printf("%f\t", L0_Arm[i]);
    // }
    // std::printf("\n");

    // for (int i = 0; i < 3; i++) {
    //     std::printf("%f\t", L0_Elbow[i]);
    // }
    // std::printf("\n");

    // for (int i = 0; i < 3; i++) {
    //     std::printf("%f\t", L0_Wrist[i]);
    // }
    // std::printf("\n");


    // position
    for (int i = 0; i < 3; i++) {
        P_06[i] = L0_Base[i] + L0_Arm[i] + L0_Elbow[i] - L0_Wrist[i];
        // std::printf("P_06[%d]:%f\n", i, P_06[i]);
    }

    

    // posture
    Rotation2Euler(R_06, &P_06[3]);

    outputPose.x = P_06[0];
    outputPose.y = P_06[1];
    outputPose.z = P_06[2];
    outputPose.roll = P_06[3] * RAD_TO_DEG;
    outputPose.pitch = P_06[4] * RAD_TO_DEG;
    outputPose.yaw = P_06[5] * RAD_TO_DEG;

    memcpy(outputPose.R, R_06,  9 * sizeof(float));

    return true;
}


bool KinematicSolver::IK(const Pose6Dof_t &inputPose, const Joint6Dof_t &lastJoint, IKSolves_t &outputSolves)
{
    float theta_1[2]{};
    float theta_23[2][2]{};
    float theta_456[2][3]{};

    float P_06[6]{};
    float P_0w[3]{};
    float L_2w[3]{};
    float R_10[9]{};
    float R_31[9]{};
    float R_30[9]{};
    float R_36[9]{};
    float R_06[9]{};

    float L0_Wrist[3]{};

    P_06[0] = inputPose.x;
    P_06[1] = inputPose.y;
    P_06[2] = inputPose.z;

    if (inputPose.vaildR) {
        memcpy(R_06, inputPose.R, 9 * sizeof(float));
    } else {
        std::printf("false");
        P_06[3] = inputPose.roll / RAD_TO_DEG;
        P_06[4] = inputPose.pitch / RAD_TO_DEG;
        P_06[5] = inputPose.yaw / RAD_TO_DEG;

        Euler2Rotation(&P_06[3], R_06);
    }

    for (int i = 0; i < 2; i++) {
        theta_1[i] = lastJoint.theta[0];
        theta_23[i][0] = lastJoint.theta[1];
        theta_23[i][1] = lastJoint.theta[2];
        theta_456[i][0] = lastJoint.theta[3];
        theta_456[i][1] = lastJoint.theta[4];
        theta_456[i][2] = lastJoint.theta[5];
    }

    MatrixMultiply(R_06, L6_Wrist_, L0_Wrist, 3, 3, 1);

    for (int i = 0; i < 3; i++) {
        P_0w[i] = P_06[i] + L0_Wrist[i];
    }

    /* ---------------------------------J1-------------------------------------- */
    if ((P_0w[0] * P_0w[0] + P_0w[1] * P_0w[1]) <= 0.000001f) {
        theta_1[0] = lastJoint.theta[0];
        theta_1[1] = theta_1[0];
        for (int i = 0; i < 4; i++) {
            outputSolves.solveFlag[0 + i][0] = -1;
            outputSolves.solveFlag[4 + i][0] = -1;
        }
    } else {
        theta_1[0] = atan2f(P_0w[1], P_0w[0]);
        theta_1[1] = atan2f(-P_0w[1], -P_0w[0]);
        for (int i = 0; i < 4; i++) {
            outputSolves.solveFlag[0 + i][0] = 1;
            outputSolves.solveFlag[4 + i][0] = 1;
        }
    }

    /* ---------------------------------J23------------------------------------- */
    for (int ind_arm = 0; ind_arm < 2; ind_arm++) {
        float ct1 = cosf(theta_1[ind_arm] + MDH_Matrix_[0][3]);
        float st1 = sinf(theta_1[ind_arm] + MDH_Matrix_[0][3]);

        R_10[0] = ct1;
        R_10[1] = st1;
        R_10[2] = 0.0f;
        R_10[3] = -st1;
        R_10[4] = ct1;
        R_10[5] = 0.0f;
        R_10[6] = 0.0f;
        R_10[7] = 0.0f;
        R_10[8] = 1.0f;

        float P_0w_r[3]{};

        // rotation
        MatrixMultiply(R_10, P_0w, P_0w_r, 3, 3, 1);

        // position
        for (int i = 0; i < 3; i++) {
            L_2w[i] = P_0w_r[i] - L0_Base_[i];
        }

        float c = sqrtf(L_2w[0]*L_2w[0] + L_2w[2]*L_2w[2]);
        float b = sqrtf(mdhConfig_.D_ELBOW * mdhConfig_.D_ELBOW + mdhConfig_.L_FOREARM * mdhConfig_.L_FOREARM);
        float a = mdhConfig_.L_UPPERARM;
        
        if (fabs(a + b - c) <= 0.000001f) {
            theta_23[0][0] = atan2f(L_2w[2], L_2w[0]);
            theta_23[1][0] = theta_23[0][0];

            theta_23[0][1] = 0.0f;
            theta_23[1][1] = 0.0f;

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
        } else if (fabs(c - fabs(a - b) <= 0.000001f)) {
            theta_23[0][0] = atan2f(L_2w[2], L_2w[0]);
            theta_23[1][0] = theta_23[0][0];

            if (ind_arm == 0) {
                theta_23[0][1] = (float)M_PI;
                theta_23[1][1] = -(float)M_PI;
            } else {
                theta_23[0][1] = -(float)M_PI;
                theta_23[1][1] = (float)M_PI;
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
            float beta2, beta3;
            float q3 = (a*a + b*b - c*c) / (2*a*b);

            if (q3 >= 1.0f) {
                beta3 = 0.0f;
            } else if (q3 <= -1.0f) {
                beta3 = (float)M_PI;
            } else {
                beta3 = acosf(q3);
            }

            float q2 = (a*a + c*c - b*b) / (2*a*c);
            if (q2 >= 1.0f) {
                beta2 = 0.0f;
            } else if (q2 <= -1.0f) {
                beta2 = (float)M_PI;
            } else {
                beta2 = acosf(q2);
            }

            float alpha2 = atan2f(L_2w[2], L_2w[0]);
            float alpha3 = atan2f(mdhConfig_.L_FOREARM, mdhConfig_.D_ELBOW);

            if (ind_arm == 0) {
                theta_23[0][0] = alpha2 + beta2;
                theta_23[0][1] = -((float)M_PI - alpha3 - beta3);
                theta_23[1][0] = -(alpha2 + beta2);
                theta_23[1][1] = (float)M_PI - alpha3 - beta3;
            } else {
                theta_23[0][0] = -(alpha2 + beta2);
                theta_23[0][1] = (float)M_PI - alpha3 - beta3;
                theta_23[1][0] = alpha2 + beta2;
                theta_23[1][1] = -((float)M_PI - alpha3 - beta3);
            }
            for (int i = 0; i < 2; i++) {
                outputSolves.solveFlag[0 + i + 4 * ind_arm][1] = 1;
                outputSolves.solveFlag[2 + i + 4 * ind_arm][1] = 1;
            }
        }
        for (int ind_elbow = 0; ind_elbow < 2; ind_elbow++) {
            float c2 = cosf(theta_23[ind_elbow][0] + MDH_Matrix_[1][3]);
            float s2 = sinf(theta_23[ind_elbow][0] + MDH_Matrix_[1][3]);
            float c3 = cosf(theta_23[ind_elbow][1] + MDH_Matrix_[2][3]);
            float s3 = sinf(theta_23[ind_elbow][1] + MDH_Matrix_[2][3]);

            R_31[0] = c2 * c3 - s2 * s3;
            R_31[1] = 0.0f;
            R_31[2] = -(c2 * s3 + s2 * c3);
            R_31[3] = -(c2 * s3 + s2 * c3);
            R_31[4] = 0.0f;
            R_31[5] = -(c2 * c3 - s2 * s3);
            R_31[6] = 0.0f;
            R_31[7] = 1.0f;
            R_31[8] = 0.0f;

            MatrixMultiply(R_31, R_10, R_30, 3, 3, 3);
            MatrixMultiply(R_30, R_06, R_36, 3, 3, 3);

            float cos5;
            if (R_36[5] > 1.0f - 0.00001f) {
                cos5 = 1.0f;
                theta_456[0][1] = 0.0f;
                theta_456[1][1] = 0.0f;
            } else if (R_36[5] < -1.0f + 0.00001f) {
                cos5 = -1.0f;
                if (ind_arm == 0) {
                    theta_456[0][1] = (float)M_PI;
                    theta_456[1][1] = -(float)M_PI;
                } else {
                    theta_456[0][1] = -(float)M_PI;
                    theta_456[1][1] = (float)M_PI;
                }
            } else {
                cos5 = R_36[5];
                if (ind_arm == 0) {
                    theta_456[0][1] = acosf(cos5);
                    theta_456[1][1] = -acosf(cos5);
                } else {
                    theta_456[0][1] = -acosf(cos5);
                    theta_456[1][1] = acosf(cos5);
                }
            }
            if (cos5 == 1.0f || cos5 == -1.0f) {
                if (ind_arm == 0) {
                    theta_456[0][0] = lastJoint.theta[3];
                    float c4 = cosf(lastJoint.theta[3] + MDH_Matrix_[3][3]);
                    float s4 = sinf(lastJoint.theta[3] + MDH_Matrix_[3][3]);
                    theta_456[0][2] = atan2f(-s4*R_36[0] - c4*R_36[6], -s4*R_36[1] - c4*R_36[7]);

                    theta_456[1][2] = lastJoint.theta[5];
                    float c6 = cosf(lastJoint.theta[5] + MDH_Matrix_[5][3]);
                    float s6 = sinf(lastJoint.theta[5] + MDH_Matrix_[5][3]);
                    theta_456[1][0] = atan2f(-s6*R_36[0] - c6*R_36[1], -s6*R_36[6] - c6*R_36[7]);
                } else {
                    theta_456[0][2] = lastJoint.theta[5];
                    float c6 =cosf(lastJoint.theta[5] + MDH_Matrix_[5][3]);
                    float s6 =sinf(lastJoint.theta[5] + MDH_Matrix_[5][3]);
                    theta_456[0][0] = atan2f(-s6*R_36[0] - c6*R_36[1], -s6*R_36[6] - c6*R_36[7]);

                    theta_456[1][0] = lastJoint.theta[3];
                    float c4 =cosf(lastJoint.theta[3] + MDH_Matrix_[3][3]);
                    float s4 =sinf(lastJoint.theta[3] + MDH_Matrix_[3][3]);
                    theta_456[1][2] = atan2f(-s4*R_36[0] - c4*R_36[6], -s4*R_36[1] - c4*R_36[7]);

                }

                outputSolves.solveFlag[4 * ind_arm + 2 * ind_elbow + 0][2] = -1;
                outputSolves.solveFlag[4 * ind_arm + 2 * ind_elbow + 1][2] = -1;
            } else {
                if (ind_arm == 0) {
                    theta_456[0][0] = atan2f(R_36[8], -R_36[2]);
                    theta_456[1][0] = atan2f(-R_36[8], R_36[2]);
                    theta_456[0][2] = atan2f(-R_36[4], R_36[3]);
                    theta_456[1][2] = atan2f(R_36[4], -R_36[3]);
                } else {
                    theta_456[1][0] = atan2f(R_36[8], -R_36[2]);
                    theta_456[0][0] = atan2f(-R_36[8], R_36[2]);
                    theta_456[1][2] = atan2f(-R_36[4], R_36[3]);
                    theta_456[0][2] = atan2f(R_36[4], -R_36[3]);
                }
                outputSolves.solveFlag[4 * ind_arm + 2 * ind_elbow + 0][2] = 1;
                outputSolves.solveFlag[4 * ind_arm + 2 * ind_elbow + 1][2] = 1;
            }

            for (int ind_wrist = 0; ind_wrist < 2; ind_wrist++) {
                while (theta_1[ind_arm] > (float)M_PI) {
                    theta_1[ind_arm] -= 2.0f * (float)M_PI;
                }
                        
                while (theta_1[ind_arm] <= -(float)M_PI) {
                    theta_1[ind_arm] += 2.0f * (float)M_PI;
                }
                outputSolves.solves[ind_arm * 4 + ind_elbow * 2 + ind_wrist].theta[0] = theta_1[ind_arm];

                for(int i = 0; i < 2; i++) {
                    while (theta_23[ind_elbow][i] > (float)M_PI) {
                        theta_23[ind_elbow][i] -= 2.0f * (float)M_PI;
                    }
                    while (theta_23[ind_elbow][i] <= -(float)M_PI) {
                        theta_23[ind_elbow][i] += 2.0f * (float)M_PI;
                    }
                    outputSolves.solves[ind_arm * 4 + ind_elbow * 2 + ind_wrist].theta[1+i] = theta_23[ind_elbow][i];

                }

                for(int i = 0; i < 3; i++) {
                    while (theta_456[ind_wrist][i] > (float)M_PI) {
                        theta_456[ind_wrist][i] -= 2.0f * (float)M_PI;
                    }
                    while (theta_456[ind_wrist][i] <= -(float)M_PI) {
                        theta_456[ind_wrist][i] += 2.0f * (float)M_PI;
                    }
                    outputSolves.solves[ind_arm * 4 + ind_elbow * 2 + ind_wrist].theta[3+i] = theta_456[ind_wrist][i];
                }
            }
        }
    }
    for (int i = 0; i < 8; i++) {
        for (int j = 0; j < 6; j++) {
            outputSolves.solves[i].theta[j] *= RAD_TO_DEG;
        }
    }

    return true;

}