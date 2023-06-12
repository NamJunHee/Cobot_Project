#ifndef IMPEDANCE_CONTROL_DELTA_H
#define IMPEDANCE_CONTROL_DELTA_H

#include <stdio.h>
#include <iostream>
#include <time.h>
#include <Eigen/Dense>
#include <cmath>
#include <QtMath>
#include <complex.h>
//#include <mainwindow.h>

#define Init_mode       0
#define Impedance_mode  1
#define IK_mode         2
#define Trajectory_mode 3
#define PI 3.141592

using namespace std;
using namespace Eigen;

class Impedance_Control_Delta
{
public:
    Impedance_Control_Delta();
public:

    double load1,load2,load3;
    Vector3d load_vector;
    Vector3d F_ext_before;
    Vector3d F_ext_temp;
    Vector3d p_d_temp;
    Vector3d x_d_temp;
    Vector3d F_ext_mobile;
    //Vector3d x_d_before;
    double time_cnt1;


    // One dof
    double g,  K ,B ,M, H, t, t3, m_ball;

    MatrixXd J_one;
    // Delta
    double tau1, tau2, tau3, gear_ratio, r_e, r_b, L1, L1_c, L2, e_x, e_y, e_z, m1, m2, mp, I_1, I_m, I_G, pi1, pi2, pi3,g11,g12,g13,g21,g22,g23,x_test,y_test,z_test;
    double b11, b12, b13, b21, b22, b23, b31, b32, b33,k1,k2,k3,theta31,theta32,theta33, theta21,theta22, theta23,theta11,theta12,theta13, dt, time_cnt, m;
    double b11_d, b12_d, b13_d, b21_d, b22_d, b23_d, b31_d, b32_d, b33_d,k1_d,k2_d,k3_d,theta31_d,theta32_d,theta33_d, theta21_d,theta22_d, theta23_d,g11_d,g12_d,g13_d,g21_d,g22_d,g23_d,theta11_d,theta12_d,theta13_d;

    Vector3d q, dq, ddq, q_d, dq_d, ddq_d, q_d_before, dq_d_before, ddq_d_before, p, p_temp,p_dot, p_dot_temp,p_ddot,p_1, p_2, p_3, theta1, theta1_dot, theta1_dot_temp,theta1_ddot, theta2,theta2_dot, theta3,theta3_dot, ddx_d, dx_d,  F_ext,tau_ext,tau_ext_temp,tau_ext_LPF;
    Vector3d w11, w12, w13, w21, w22, w23, w31, w32, w33, u11, u12, u13, u21, u22, u23;
    Vector3d p_1_dot, p_2_dot, p_3_dot, j_q, jq_dot, tau_delta, v_1, v_2,v_3,dx_d_before,x_d_before, theta1_d,theta1_d_out,p_d, theta1_d_temp, gravity_temp;
    Vector3d x_d, m_est;
    double j1x, j2x, j3x, j1y, j2y, j3y, j1z, j2z, j3z, j1x_dot, j2x_dot, j3x_dot, j1y_dot, j2y_dot, j3y_dot, j1z_dot, j2z_dot, j3z_dot;
    MatrixXd J_q, J_x, J;
    MatrixXd J_xdot, Jq_dot, I_3, M_d,C, p_pdi, p_theta, D_temp;
    DiagonalMatrix<double, 3> D;

    double J_a, J_A, J_1, m_m, l1c;

    Vector3d ww, ww31,ww32,ww33; //[0;1;0]
    // Forward kinematics
    Vector3d P_b_1, P_b_2, P_b_3, shaft_1, shaft_2, shaft_3, P_e_1, P_e_2, P_e_3, P_e, P_d1, P_d2, P_d3, P_d11, P_d22, P_d33,D3E_temp;
    Vector3d On, v1, Zn, v2, v3, Yn, Xn, G,D_1,D_2,D_3, p_inv,p_traj,G_comp;
    Vector3d p_d_before, p_;
    double protection_1,protection_2,protection_3,l_hat1,l_hat2,l_hat3,d_1,d_2,d_3;  

    double 	R12, D3E, R3, x3, x_hat, y_hat;

    MatrixXd P_n, T, P_b;

    int F_flag1,F_flag2,F_flag3, mode_switch;

    double x_traj,y_traj,z_traj;

    double DegreesToRadians(double degrees);
    MatrixXd RotateMatrix(double pi);
    MatrixXd Torque_Cal_Delta(double loadcell1, double loadcell2, double loadcell3, double encoder1, double encoder2, double encoder3);
    Vector3d Delta_Forward_kinematics(double theta1, double theta2, double theta3);
    Vector3d Filtering_loadcell(double loadcell1, double loadcell2, double loadcell3);
    Vector3d LPF_Position(double cutoff_frequency, double dt,Vector3d  x_d_temp_ , Vector3d x_d_before_);
    Vector3d LPF_Force(double cutoff_frequency, double dt,Vector3d Force_ , Vector3d force_before);

    void Delta_inverse_kinematics(Vector3d p,double encoder1, double encoder2, double encoder3);
    void Cal_theta2();
    void Cal_theta3();
    void Cal_p_i_dot();
    void Cal_Jacobian();
    void Cal_theta_dot();
    void Cal_Jacobian_dot();
    void Cal_theta1_d();
    void Trajectory();
    void Gravity_Compensation();

    double k_gain;

    double cutoff_frequency;



};

#endif // IMPEDANCE_CONTROL_DELTA_H
