#include "delta_forward_kinematics.h"

delta_forward_kinematics::delta_forward_kinematics()
{
    Base_len =  0.1;
    End_Base_len = 0.05;

    L1_len = 0.2;
    L2_len = 0.368;

    B_origin = MatrixXd(3,1);

    B1.L1_origin = MatrixXd(3,1);
    B2.L1_origin = MatrixXd(3,1);
    B3.L1_origin = MatrixXd(3,1);

    B1.L1_shift_vale = MatrixXd(3,1);
    B2.L1_shift_vale = MatrixXd(3,1);
    B3.L1_shift_vale = MatrixXd(3,1);

    X_rotate = MatrixXd(3,3);
    Y_rotate = MatrixXd(3,3);
    Z_rotate = MatrixXd(3,3);

    B1.L1_r_theta = deg2rad(120);
    B2.L1_r_theta = deg2rad(120);
    B3.L1_r_theta = deg2rad(120);


}

double delta_forward_kinematics::F_kinematics(double theta_1, double theta_2, double theta_3)
{
    B1.L1_theta = theta_1;
    B2.L1_theta = theta_2;
    B3.L1_theta = theta_3;

    B_origin << 0,
            0,
            0;

    B1.L1_shift_vale << -Base_len,
            0,
            0;
    B2.L1_shift_vale << Base_len,
            0,
            0;
    B3.L1_shift_vale << 0,
            -Base_len,
            0;


    Z_rotate << cos(B1.L1_r_theta), -sin(B1.L1_r_theta), 0,
            sin(B1.L1_r_theta), cos(B1.L1_r_theta),0,
            0,0,1;

    B1.L1_origin  = Z_rotate * ( B_origin + B1.L1_shift_vale);

    Z_rotate << cos(B2.L1_r_theta), -sin(B2.L1_r_theta), 0,
            sin(B2.L1_r_theta), cos(B2.L1_r_theta),0,
            0,0,1;

    B2.L1_origin  = Z_rotate * ( B_origin + B2.L1_shift_vale );

    B3.L1_origin  = B_origin + B1.L1_shift_vale;

    //cout << "B1_origin >> " << B1.L1_origin << endl;
    //cout << "B2_origin >> " << B2.L1_origin << endl;
    //cout << "B3_origin >> " << B3.L1_origin << endl;

    return 1.1;
}

double delta_forward_kinematics::deg2rad(double degree)
{
    return degree*PI/180;
}

