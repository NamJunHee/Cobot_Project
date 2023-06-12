#ifndef DELTA_FORWARD_KINEMATICS_H
#define DELTA_FORWARD_KINEMATICS_H

#include <stdio.h>
#include <iostream>
#include <time.h>
#include <Eigen/Dense>
#include <cmath>
#include <QtMath>

#define PI 3.141592

using namespace std;
using namespace Eigen;

class delta_forward_kinematics
{
public:
    double Base_len;
    double End_Base_len;

    double L1_len;
    double L2_len;

    MatrixXd B_origin;

    Matrix3d X_rotate;
    Matrix3d Y_rotate;
    Matrix3d Z_rotate;


    struct Base{
        double L1_theta;
        MatrixXd L1_shift_vale;
        int L1_r_theta;

        MatrixXd L1_origin;
        MatrixXd L2_origin;
    };

    Base B1;
    Base B2;
    Base B3;

    delta_forward_kinematics();
    double F_kinematics(double theta_1, double theta_2, double theta_3);
    double deg2rad(double degree);

};

#endif // DELTA_FORWARD_KINEMATICS_H
