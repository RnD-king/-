#include "IK_Solver.hpp"
#include <eigen3/Eigen/Dense>
#include <cmath>
#include <iostream>

using Eigen::Matrix4d;
using Eigen::VectorXd;
using Eigen::MatrixXd;

// DH Transformation - just reference, Not used here, but good for generalization
Eigen::Matrix4d DH(double a, double alpha, double d, double theta) {
    Matrix4d T;
    T << cos(theta), -sin(theta), 0, a,
         sin(theta)*cos(alpha), cos(theta)*cos(alpha), -sin(alpha), -sin(alpha)*d,
         sin(theta)*sin(alpha), cos(theta)*sin(alpha), cos(alpha), cos(alpha)*d,
         0, 0, 0, 1;
    return T;
}

Eigen::VectorXd BRP_LL_IK(const VectorXd& Ref_LF_PR, const VectorXd& InitAng, double L0, double L3, double L4, double L6)
{
    double del_Q = 0.0001;
    VectorXd th = InitAng;
    VectorXd th_IK(6);
    int max_iter = 100;

    for (int iter = 0; iter < max_iter; ++iter)
    {
        VectorXd old_Q = th;
        
        // ----- Forward Kinematics -----
        // 각 관절변수별 변환행렬
        Matrix4d LL_A1, LL_A2, LL_A3, LL_A4, LL_A5, LL_A6, LL_A7, LL_A8;
        LL_A1 << 0, 1, 0, 0,  -1, 0, 0, L0,  0, 0, 1, 0,  0, 0, 0, 1;
        LL_A2 << cos(th(0)), 0, -sin(th(0)), 0,
                 sin(th(0)), 0,  cos(th(0)), 0,
                 0,         -1, 0,          0,
                 0,          0, 0,          1;
        LL_A3 << -sin(th(1)), 0, -cos(th(1)), 0,
                  cos(th(1)), 0, -sin(th(1)), 0,
                  0,         -1, 0,           0,
                  0,          0, 0,           1;
        LL_A4 << cos(th(2)), -sin(th(2)), 0, L3*cos(th(2)),
                 sin(th(2)),  cos(th(2)), 0, L3*sin(th(2)),
                 0,           0,          1, 0,
                 0,           0,          0, 1;
        LL_A5 << cos(th(3)), -sin(th(3)), 0, L4*cos(th(3)),
                 sin(th(3)),  cos(th(3)), 0, L4*sin(th(3)),
                 0,           0,          1, 0,
                 0,           0,          0, 1;
        LL_A6 << cos(th(4)), 0, sin(th(4)), 0,
                 sin(th(4)), 0, -cos(th(4)), 0,
                 0,          1, 0,           0,
                 0,          0, 0,           1;
        LL_A7 << cos(th(5)), 0, -sin(th(5)), L6*cos(th(5)),
                 sin(th(5)), 0,  cos(th(5)), L6*sin(th(5)),
                 0,         -1, 0,           0,
                 0,          0, 0,           1;
        LL_A8 << 0, 0, -1, 0,
                 -1, 0, 0, 0,
                  0, 1, 0, 0,
                  0, 0, 0, 1;
        Matrix4d LL_T_01 = LL_A1;
        Matrix4d LL_T_02 = LL_T_01 * LL_A2;
        Matrix4d LL_T_03 = LL_T_02 * LL_A3;
        Matrix4d LL_T_04 = LL_T_03 * LL_A4;
        Matrix4d LL_T_05 = LL_T_04 * LL_A5;
        Matrix4d LL_T_06 = LL_T_05 * LL_A6;
        Matrix4d LL_T_07 = LL_T_06 * LL_A7;
        Matrix4d LL_T_08 = LL_T_07 * LL_A8;     // <- end-effector coordinate

        // End-effector 위치/자세
        VectorXd PR(6);
        PR(0) = LL_T_08(0, 3); // x
        PR(1) = LL_T_08(1, 3); // y
        PR(2) = LL_T_08(2, 3); // z

        double nx = LL_T_08(0, 0), ny = LL_T_08(1, 0), nz = LL_T_08(2, 0);
        double ox = LL_T_08(0, 1), oy = LL_T_08(1, 1), oz = LL_T_08(2, 1);
        double ax = LL_T_08(0, 2), ay = LL_T_08(1, 2), az = LL_T_08(2, 2);

        PR(5) = atan2(ny, nx); // rol - z
        PR(4) = atan2(-nz, cos(PR(5))*nx + sin(PR(5))*ny); // pitch - y
        PR(3) = atan2(sin(PR(5))*ax - cos(PR(5))*ay, -sin(PR(5))*ox + cos(PR(5))*oy); // yaw - x 

        // Error Vector F
        VectorXd F = Ref_LF_PR - PR;
        double ERR = F.norm();

        // 수렴 검사
        if (ERR < 0.0001) {
            th_IK = th;
            break;
        }
        else if (iter == max_iter - 1) {
            th_IK = InitAng;
            break;
        }

        // Numerical Jacobian Calculation
        MatrixXd J(6, 6);
        for (int i = 0; i < 6; ++i) {
            VectorXd th_pert = th;
            th_pert(i) += del_Q;

            // Perturbed FK
            Matrix4d A2, A3, A4, A5, A6, A7;
            A2 << cos(th_pert(0)), 0, -sin(th_pert(0)), 0,
                  sin(th_pert(0)), 0,  cos(th_pert(0)), 0,
                  0,         -1, 0,          0,
                  0,          0, 0,          1;
            A3 << -sin(th_pert(1)), 0, -cos(th_pert(1)), 0,
                   cos(th_pert(1)), 0, -sin(th_pert(1)), 0,
                   0,         -1, 0,           0,
                   0,          0, 0,           1;
            A4 << cos(th_pert(2)), -sin(th_pert(2)), 0, L3*cos(th_pert(2)),
                  sin(th_pert(2)),  cos(th_pert(2)), 0, L3*sin(th_pert(2)),
                  0,           0,          1, 0,
                  0,           0,          0, 1;
            A5 << cos(th_pert(3)), -sin(th_pert(3)), 0, L4*cos(th_pert(3)),
                  sin(th_pert(3)),  cos(th_pert(3)), 0, L4*sin(th_pert(3)),
                  0,           0,          1, 0,
                  0,           0,          0, 1;
            A6 << cos(th_pert(4)), 0, sin(th_pert(4)), 0,
                  sin(th_pert(4)), 0, -cos(th_pert(4)), 0,
                  0,          1, 0,           0,
                  0,          0, 0,           1;
            A7 << cos(th_pert(5)), 0, -sin(th_pert(5)), L6*cos(th_pert(5)),
                  sin(th_pert(5)), 0,  cos(th_pert(5)), L6*sin(th_pert(5)),
                  0,         -1, 0,           0,
                  0,          0, 0,           1;
            
            Matrix4d T_01 = LL_A1;
            Matrix4d T_02 = T_01 * A2;
            Matrix4d T_03 = T_02 * A3;
            Matrix4d T_04 = T_03 * A4;
            Matrix4d T_05 = T_04 * A5;
            Matrix4d T_06 = T_05 * A6;
            Matrix4d T_07 = T_06 * A7;
            Matrix4d T_08 = T_07 * LL_A8;

            VectorXd PR_pert(6);
            PR_pert(0) = T_08(0, 3);
            PR_pert(1) = T_08(1, 3);
            PR_pert(2) = T_08(2, 3);

            double nxp = T_08(0, 0), nyp = T_08(1, 0), nzp = T_08(2, 0);
            double oxp = T_08(0, 1), oyp = T_08(1, 1), ozp = T_08(2, 1);
            double axp = T_08(0, 2), ayp = T_08(1, 2), azp = T_08(2, 2);

            PR_pert(5) = atan2(nyp, nxp); // roll
            PR_pert(4) = atan2(-nzp, cos(PR_pert(5))*nxp + sin(PR_pert(5))*nyp);
            PR_pert(3) = atan2(sin(PR_pert(5))*axp - cos(PR_pert(5))*ayp, -sin(PR_pert(5))*oxp + cos(PR_pert(5))*oyp);

            J.col(i) = (PR_pert - PR) / del_Q;
        }

        // Joint Angle Update (뉴턴-랩슨 방식)
        th = old_Q + J.inverse() * F;

        // 무릎 각도 제약
        if (th(3) < 0) th(3) = -th(3); // Knee index = 3
    }

    return th_IK;
}




Eigen::VectorXd BRP_RL_IK(const VectorXd& Ref_RF_PR, const VectorXd& InitAng, double L0, double L3, double L4, double L6)
{
    double del_Q = 0.0001;
    VectorXd th = InitAng;
    VectorXd th_IK(6);
    int max_iter = 100;

    for (int iter = 0; iter < max_iter; ++iter)
    {
        VectorXd old_Q = th;
        // ----- Forward Kinematics -----
        // 각 관절변수별 변환행렬
        Matrix4d RL_A1, RL_A2, RL_A3, RL_A4, RL_A5, RL_A6, RL_A7, RL_A8;
        RL_A1 << 0, 1, 0, 0,  -1, 0, 0, -L0,  0, 0, 1, 0,  0, 0, 0, 1;
        RL_A2 << cos(th(0)), 0, -sin(th(0)), 0,
                 sin(th(0)), 0,  cos(th(0)), 0,
                 0,         -1, 0,          0,
                 0,          0, 0,          1;
        RL_A3 << -sin(th(1)), 0, -cos(th(1)), 0,
                  cos(th(1)), 0, -sin(th(1)), 0,
                  0,         -1, 0,           0,
                  0,          0, 0,           1;
        RL_A4 << cos(th(2)), -sin(th(2)), 0, L3*cos(th(2)),
                 sin(th(2)),  cos(th(2)), 0, L3*sin(th(2)),
                 0,           0,          1, 0,
                 0,           0,          0, 1;
        RL_A5 << cos(th(3)), -sin(th(3)), 0, L4*cos(th(3)),
                 sin(th(3)),  cos(th(3)), 0, L4*sin(th(3)),
                 0,           0,          1, 0,
                 0,           0,          0, 1;
        RL_A6 << cos(th(4)), 0, sin(th(4)), 0,
                 sin(th(4)), 0, -cos(th(4)), 0,
                 0,          1, 0,           0,
                 0,          0, 0,           1;
        RL_A7 << cos(th(5)), 0, -sin(th(5)), L6*cos(th(5)),
                 sin(th(5)), 0,  cos(th(5)), L6*sin(th(5)),
                 0,         -1, 0,           0,
                 0,          0, 0,           1;
        RL_A8 << 0, 0, -1, 0,
                 -1, 0, 0, 0,
                  0, 1, 0, 0,
                  0, 0, 0, 1;
        Matrix4d RL_T_01 = RL_A1;
        Matrix4d RL_T_02 = RL_T_01 * RL_A2;
        Matrix4d RL_T_03 = RL_T_02 * RL_A3;
        Matrix4d RL_T_04 = RL_T_03 * RL_A4;
        Matrix4d RL_T_05 = RL_T_04 * RL_A5;
        Matrix4d RL_T_06 = RL_T_05 * RL_A6;
        Matrix4d RL_T_07 = RL_T_06 * RL_A7;
        Matrix4d RL_T_08 = RL_T_07 * RL_A8;     // <- end-effector coordinate

        // End-effector 위치/자세
        VectorXd PR(6);
        PR(0) = RL_T_08(0, 3); // x
        PR(1) = RL_T_08(1, 3); // y
        PR(2) = RL_T_08(2, 3); // z

        double nx = RL_T_08(0, 0), ny = RL_T_08(1, 0), nz = RL_T_08(2, 0);
        double ox = RL_T_08(0, 1), oy = RL_T_08(1, 1), oz = RL_T_08(2, 1);
        double ax = RL_T_08(0, 2), ay = RL_T_08(1, 2), az = RL_T_08(2, 2);

        PR(5) = atan2(ny, nx); // roll
        PR(4) = atan2(-nz, cos(PR(5))*nx + sin(PR(5))*ny); // pitch
        PR(3) = atan2(sin(PR(5))*ax - cos(PR(5))*ay, -sin(PR(5))*ox + cos(PR(5))*oy); // yaw

        // Error Vector F
        VectorXd F = Ref_RF_PR - PR;
        double ERR = F.norm();

        // 수렴 test
        if (ERR < 0.0001) {
            th_IK = th;
            break;
        }
        else if (iter == max_iter - 1) {
            th_IK = InitAng;
            break;
        }

        // Numerical Jacobian Calculation
        MatrixXd J(6, 6);
        for (int i = 0; i < 6; ++i) {
            VectorXd th_pert = th;
            th_pert(i) += del_Q;

            // Perturbed FK
            Matrix4d A2, A3, A4, A5, A6, A7;
            A2 << cos(th_pert(0)), 0, -sin(th_pert(0)), 0,
                  sin(th_pert(0)), 0,  cos(th_pert(0)), 0,
                  0,         -1, 0,          0,
                  0,          0, 0,          1;
            A3 << -sin(th_pert(1)), 0, -cos(th_pert(1)), 0,
                   cos(th_pert(1)), 0, -sin(th_pert(1)), 0,
                   0,         -1, 0,           0,
                   0,          0, 0,           1;
            A4 << cos(th_pert(2)), -sin(th_pert(2)), 0, L3*cos(th_pert(2)),
                  sin(th_pert(2)),  cos(th_pert(2)), 0, L3*sin(th_pert(2)),
                  0,           0,          1, 0,
                  0,           0,          0, 1;
            A5 << cos(th_pert(3)), -sin(th_pert(3)), 0, L4*cos(th_pert(3)),
                  sin(th_pert(3)),  cos(th_pert(3)), 0, L4*sin(th_pert(3)),
                  0,           0,          1, 0,
                  0,           0,          0, 1;
            A6 << cos(th_pert(4)), 0, sin(th_pert(4)), 0,
                  sin(th_pert(4)), 0, -cos(th_pert(4)), 0,
                  0,          1, 0,           0,
                  0,          0, 0,           1;
            A7 << cos(th_pert(5)), 0, -sin(th_pert(5)), L6*cos(th_pert(5)),
                  sin(th_pert(5)), 0,  cos(th_pert(5)), L6*sin(th_pert(5)),
                  0,         -1, 0,           0,
                  0,          0, 0,           1;
            Matrix4d T_01 = RL_A1;
            Matrix4d T_02 = T_01 * A2;
            Matrix4d T_03 = T_02 * A3;
            Matrix4d T_04 = T_03 * A4;
            Matrix4d T_05 = T_04 * A5;
            Matrix4d T_06 = T_05 * A6;
            Matrix4d T_07 = T_06 * A7;
            Matrix4d T_08 = T_07 * RL_A8;

            VectorXd PR_pert(6);
            PR_pert(0) = T_08(0, 3);
            PR_pert(1) = T_08(1, 3);
            PR_pert(2) = T_08(2, 3);

            double nxp = T_08(0, 0), nyp = T_08(1, 0), nzp = T_08(2, 0);
            double oxp = T_08(0, 1), oyp = T_08(1, 1), ozp = T_08(2, 1);
            double axp = T_08(0, 2), ayp = T_08(1, 2), azp = T_08(2, 2);

            PR_pert(5) = atan2(nyp, nxp); // roll
            PR_pert(4) = atan2(-nzp, cos(PR_pert(5))*nxp + sin(PR_pert(5))*nyp);
            PR_pert(3) = atan2(sin(PR_pert(5))*axp - cos(PR_pert(5))*ayp, -sin(PR_pert(5))*oxp + cos(PR_pert(5))*oyp);

            J.col(i) = (PR_pert - PR) / del_Q;
        }

        // Joint Angle Update (뉴턴-랩슨 방식)
        th = old_Q + J.inverse() * F;

        // 무릎 각도 제약
        if (th(3) < 0) th(3) = -th(3); // Knee index=3
    }

    return th_IK;
}