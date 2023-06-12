#ifndef IMPEDANCE_CONTROL_H
#define IMPEDANCE_CONTROL_H

#include <stdio.h>
#include <iostream>
#include <time.h>
#include <Eigen/Dense>
#include <cmath>
#include <QtMath>

#define PI 3.141592

using namespace std;
using namespace Eigen;


class Impedance_Control
{
    public:
        double g, m, L, tau, init_q, init_dq,  dq, ddq, t,q_d,dq_d,ddq_d,q_d_before,dq_d_before,ddq_d_before, K,B, M , H, G, dq_before,G_load, m_load,q1, tau_load,tau_gravity, L_load, L_m, tau2, tau_ext1,ddq1,tau_ext_LPF;
        double cnt = 0;
        double q_before = 0;
        double tau1 = 0;
        double err_vel;
        double input_array[20] ={0,};
        double loadcell_array[20] = {0,};
        int i;
        int n = 20;
        int average = 0;
        double time_cnt = 0;

        double tau_ext_temp = 0;
        int F_flag = 0;



        MatrixXd F_ext, ddx_d, dx_d, x_d, x, J, ddx_d_before, dx_d_before, x_d_before;

        Impedance_Control();
        double Torque_calculator(double tau_ext, double q);
        double Maf(double *input_array, int n);
        double LPF(double *input_array);

    private:

};

#endif // IMPEDANCE_CONTROL_H
