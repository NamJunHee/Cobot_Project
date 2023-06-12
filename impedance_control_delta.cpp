#include "impedance_control_delta.h"

Impedance_Control_Delta::Impedance_Control_Delta()
{
    g = 9.8148;
    // 현재 각도, 각속도, 각가속도
    q << 0, 0, 0;
    dq << 0, 0, 0;
    ddq << 0, 0, 0;
    // 목표 각도, 각속도, 각가속도
    q_d << 0, 0, 0;
    dq_d << 0, 0, 0;
    ddq_d << 0, 0, 0;
    // 이전 각도, 각속도, 각가속도
    q_d_before << 0, 0, 0;
    dq_d_before << 0, 0, 0;
    ddq_d_before << 0, 0, 0;
    // 목표 위치, 속도, 가속도
    ddx_d = MatrixXd(3, 1);
    dx_d = MatrixXd(3, 1);
    x_d = MatrixXd(3, 1);
    J_one = MatrixXd(3, 1);

    pi1 = PI/3;
    pi2 = PI;
    pi3 = -PI/3;

    //Delta Variable
    // 출력 토크
    //tau1, tau2, tau3 = 0;

    gear_ratio = 1 / 9;
    r_e = 0.1;              // End effect radius
    r_b = 0.05;             // Base radius
    L1 = 0.2;               // upper link
    L1_c = 0.1;              // l1의 무게중심
    L2 = 0.368;             // lower link
/*
    e_x = 0.0;             // end_effector의 x
    e_y = 0.0;             // end_effector의 y
    e_z = 0.5;               // end_effector의 z
    // 현재 위치 좌표
    p << e_x, e_y, e_z;*/

    m1 = 0.282;           // 링크1 질량
    m2 = 0.1316;           // 링크2 질량
    mp = 0.11;             // plate 질량
    m_ball = 0.017;        // ball joint 1 개

    I_1 = 0.00289;

    I_m = 0.00000001;
    I_G = 0.00000001;

    ww << 0,
         -1,
          0;

    // 각속도
    theta1_dot << 0,
                  0,
                  0;

    theta2_dot << 0,
                  0,
                  0;

    theta3_dot << 0,
                  0,
                  0;


    dt = 0.004;         //주기
    t = 0.004;
    //자코비안
    J_q = MatrixXd(3,3);
    Jq_dot = MatrixXd(3, 3);
    J_x = MatrixXd(3,3);
    J_xdot = MatrixXd(3, 3);
    J = MatrixXd(3, 3);
    p_pdi = MatrixXd(3, 3);
    p_theta = MatrixXd(3,3);
    D_temp = MatrixXd(3,3);

    P_n = MatrixXd(4, 1);
    T = MatrixXd(4, 4);
    time_cnt1 = 0;
    gear_ratio = 1 / 9;
    J_1 = 0.00289;
    J_a = 0;
    J_A = J_1 + m2 * L1 *L1;//+gear_ratio * gear_ratio * J_a;
    m_m = mp + 3 * m2;

    I_3 = Matrix3d(3, 3);
    M_d = Matrix3d(3, 3);
    C = Matrix3d(3, 3);

    I_3 << 1, 0, 0,
           0, 1, 0,
           0, 0, 1;
    l1c = 0.0591;
    time_cnt = 0;

    x_test = 30;
    y_test = 60;
    z_test = 60;

    P_b_1 << r_b * cos(2 * (1 - 1) * PI / 3), r_b * sin(2 * (1 - 1) * PI / 3), 0;
    P_b_2 << r_b * cos(2 * (2 - 1) * PI / 3), r_b * sin(2 * (2 - 1) * PI / 3), 0;
    P_b_3 << r_b * cos(2 * (3 - 1) * PI / 3), r_b * sin(2 * (3 - 1) * PI / 3), 0;


    shaft_1 << -sin(2 * (1 - 1) * PI / 3), cos(2 * (1 - 1) * PI / 3), 0;
    shaft_2 << -sin(2 * (2 - 1) * PI / 3), cos(2 * (2 - 1) * PI / 3), 0;
    shaft_3 << -sin(2 * (3 - 1) * PI / 3), cos(2 * (3 - 1) * PI / 3), 0;


    P_e_1 << r_e * cos(2 * (1 - 1) * PI / 3), r_e * sin(2 * (1 - 1) * PI / 3), 0;
    P_e_2 << r_e * cos(2 * (2 - 1) * PI / 3), r_e * sin(2 * (2 - 1) * PI / 3), 0;
    P_e_3 << r_e * cos(2 * (3 - 1) * PI / 3), r_e * sin(2 * (3 - 1) * PI / 3), 0;

    p_inv(0) = 0.0;
    p_inv(1) = 0.0;
    p_inv(2) = 0.35;
    mode_switch = Init_mode;

    p_traj <<0,0,0;

    t3 = 0;


    v_3 << 0, 0, g;

    m = 1;
    B = 10;
    K = 0.01;
    k_gain = 1;
    cutoff_frequency = 15;

    p_d << 0,0,0.35;
    p_d_before << 0.0,0.0,0.35;
    p_ << 0,0,0.35;

    F_ext_before << 0,0,0;
    x_d_before << 0,0,0;

}

double Impedance_Control_Delta::DegreesToRadians(double degrees)
{
    return degrees * PI / 180;
}

MatrixXd Impedance_Control_Delta::RotateMatrix(double pi)
{
    MatrixXd R = MatrixXd(3, 3);

    R(0, 0) = cos(pi);    R(0, 1) = -sin(pi);    R(0, 2) = 0;
    R(1, 0) = sin(pi);    R(1, 1) = cos(pi);     R(1, 2) = 0;
    R(2, 0) = 0;          R(2, 1) = 0;           R(2, 2) = 1;

    return R;
}

MatrixXd Impedance_Control_Delta::Torque_Cal_Delta(double loadcell1, double loadcell2, double loadcell3, double encoder1, double encoder2, double encoder3)
{
    //p = Delta_Forward_kinematics(qDegreesToRadians(x_test),qDegreesToRadians(y_test),qDegreesToRadians(z_test));

    load1 = loadcell1;
    load2 = loadcell2;
    load3 = loadcell3;

    p = Delta_Forward_kinematics(encoder1,encoder2,encoder3);
    e_x = p(0);
    e_y = p(1);
    e_z = p(2);

    Cal_theta3();
    Cal_theta2();

    theta11 = encoder1;
    theta12 = encoder2;
    theta13 = encoder3;

    theta1 << theta11, theta12, theta13;

    Cal_p_i_dot();
    Cal_Jacobian();

   //p_dot = (p - p_temp) / dt;
    //p_ddot = (p_dot - p_dot_temp) / dt;

    Cal_theta_dot();
    Cal_Jacobian_dot();

    theta1_ddot = (theta1_dot - theta1_dot_temp)/t;

    int limit = 1 ;
    if(abs(loadcell1) >15 || abs(loadcell2) >15 || abs(loadcell3) >15)
    {
        loadcell1 = 0.0;
        loadcell2 = 0.0;
        loadcell3 = 0.0;

    }
    if(loadcell1 > limit || loadcell1 < -limit)
    {
        tau_ext(0) = loadcell1;
        F_flag1 = 1;
    }
    else
    {
        tau_ext(0) = 0;
        F_flag1 = 0;
    }
    if(loadcell2 > limit || loadcell2 < -limit)
    {
        tau_ext(1) = loadcell2;
        F_flag2 = 1;
    }
    else
    {
        tau_ext(1) = 0;
        F_flag2 = 0;
    }
    if(loadcell3 > limit || loadcell3 < - limit)
    {
        tau_ext(2) = loadcell3;
        F_flag3 = 1;
    }
    else
    {
        tau_ext(2) = 0;
        F_flag3 = 0;
    }

   F_ext = J.transpose() * tau_ext;
   F_ext_mobile = RotateMatrix(qDegreesToRadians(-30.0)) * J.transpose() * tau_ext;
   //F_ext = LPF_Force(cutoff_frequency, dt, F_ext_temp,F_ext_before);
   F_ext_before = F_ext;


    if(tau_ext(0) == 0 && tau_ext(1) == 0 && tau_ext(2) == 0)
    {
        //  =100;004
        time_cnt1 = 1;
        x_d << 0,0,0;
    }    
    else
    {
        time_cnt += 0.004;
        //time_cnt = 100;
        time_cnt1 += 0.002;
        //dx_d = F_ext/m*exp(-B/m*time_cnt);update
        //double mbk = (2*sin((time_cnt*sqrt(-B*B+4*K*m)nan)/(2*m))*exp(-(B*time_cnt)/(2*m)))/sqrt(-B*B+4*K*m);
        //double AA = sqrt(-B*B+4*K*m);
        //double BB = (-(B*time_cnt)/(2*m));

        //cout << "mbk : " << mbk << endl;
        //cout << "AA : " << AA << endl;
        //cout << "BB : " << BB << endl;
        //cout << "time : " << time_cnt << endl;


        complex <double> z = csqrt(-B*B+4*K*m);
        //cout << "z : " << z <<endl;

        complex <double> c1 = sin((time_cnt * z)/(2*m));
        //cout <<"c1 : " << c1 <<endl;

        complex <double> c2 = exp(-(B*time_cnt)/(2*m))/z;
        //cout << "c2 :" << c2 <<endl;


        complex <double> c3 = c1 * c2;

        //cout << "c3.real() * 2 >> " << c3.real() * 2 << endl;

        //cout << F_ext <<F_ext << endl;
        x_d_temp = F_ext * 2 * c3.real();
        //cout << "x_d : " << x_d << endl;

        //cout << "p_temp" << p_temp << endl;
        if(time_cnt1 >= 1) time_cnt1 = 0;

        x_d = LPF_Position(cutoff_frequency, dt, x_d_temp, x_d_before);
        x_d_before = x_d;

        p_temp(0) = x_d(0)*time_cnt1;
        p_temp(1) = x_d(1)*time_cnt1;

        //p_temp(0) = x_d_temp(0)*time_cnt1;
        //p_temp(1) = x_d_temp(1)*time_cnt1;

    }


    p_ << p_temp(0) + p(0),p_temp(1) + p(1),0.35;
    //cout << "p_ : " << p_ << endl;
    //p_d_temp = p_ + x_d;
    p_d = p_ + k_gain*x_d;



    if(p_d(0) > 0.12)
    {
        p_d(0) = 0.12;
    }
    else if(p_d(0) < -0.12)
    {
        p_d(0) = -0.12;
    }
    if(p_d(1) > 0.12)
    {
        p_d(1) =0.12;
    }
    else if(p_d(1) < -0.12)
    {
        p_d(1) = -0.12;
    }
    p_d(2) = 0.35;

    if(mode_switch == Impedance_mode)
    {
        Delta_inverse_kinematics(p_d,encoder1,encoder2,encoder3);
    }
    else if (mode_switch == IK_mode)
    {
        Delta_inverse_kinematics(p_inv,encoder1,encoder2,encoder3);
    }
    else if(mode_switch == Trajectory_mode)
    {
        Trajectory();
        Delta_inverse_kinematics(p_traj,encoder1,encoder2,encoder3);
        F_flag1 = 1;
        F_flag2 = 1;
        F_flag3 = 1;
    }
    else
    {
        Delta_inverse_kinematics(p,encoder1,encoder2,encoder3);

        F_flag1 = 1;
        F_flag2 = 1;
        F_flag3 = 1;
    }
    gravity_temp << 0, 0, g;

    v_1 << cos(theta11), cos(theta12), cos(theta13);
    v_2 << F_ext(0), F_ext(1), F_ext(2) + g;    // f1, , f2, f3는 외부 힘 ?

    M_d = J_A * I_3 + m_m * J.transpose().inverse()*J.transpose();
    C = m_m * J.transpose().inverse() *(-J_x.transpose().inverse() * J_xdot * J_x.inverse() * J_q + J_x.inverse() * Jq_dot);
    G = mp*J.transpose()*gravity_temp; //+  (m1 *l1c + m2* L1 ) * g * v_1;//-(m1 * l1c + m2 * L1) * g * v1;// - m_m * J.transpose().inverse()*v2;

    //mp = (m_est(0)+m_est(1)+m_est(2))/3;

    tau_delta = /*M_d * theta1_ddot + C * theta1_dot +*/ G;

    //p_temp = p;
    p_dot_temp = p_dot; // 이전 p 속도 저장
    theta1_dot_temp = theta1_dot;
    dx_d_before = dx_d;
    theta1_d_temp = theta1_d;

    return tau_delta;
}

void Impedance_Control_Delta::Delta_inverse_kinematics(Vector3d p, double encoder1, double encoder2, double encoder3)
{
    protection_1 = p.dot(P_e_1)/r_e;
    protection_2 = p.dot(P_e_2)/r_e;
    protection_3 = p.dot(P_e_3)/r_e;

    d_1 = sqrt(p(0)*p(0)+p(1)*p(1)-protection_1*protection_1);
    d_2 = sqrt(p(0)*p(0)+p(1)*p(1)-protection_2*protection_2);
    d_3 = sqrt(p(0)*p(0)+p(1)*p(1)-protection_3*protection_3);


    l_hat1 = sqrt(L2*L2 -d_1*d_1);
    l_hat2 = sqrt(L2*L2 -d_2*d_2);
    l_hat3 = sqrt(L2*L2 -d_3*d_3);

    theta11_d = atan2(protection_1-r_e,p(2)) + acos(((P_e_1-p).norm()*(P_e_1-p).norm() + L1*L1 -l_hat1*l_hat1)/(2*L1*(P_e_1 - p).norm()));
    theta12_d = atan2(protection_2-r_e,p(2)) + acos(((P_e_2-p).norm()*(P_e_2-p).norm() + L1*L1 -l_hat2*l_hat2)/(2*L1*(P_e_2 - p).norm()));
    theta13_d = atan2(protection_3-r_e,p(2)) + acos(((P_e_3-p).norm()*(P_e_3-p).norm() + L1*L1 -l_hat3*l_hat3)/(2*L1*(P_e_3 - p).norm()));

    if(theta11_d >0 && theta12_d > 0 && theta13_d >0 && theta11_d < 120*0.0174533 && theta12_d < 120*0.0174533 && theta13_d < 120*0.0174533)
    {
        theta1_d << PI/2-theta11_d, PI/2-theta12_d, PI/2-theta13_d;
    }

}

Vector3d Impedance_Control_Delta::Delta_Forward_kinematics(double theta1, double theta2, double theta3)
{
    Vector3d p;

    theta1 = PI / 2 - theta1;
    theta2 = PI / 2 - theta2;
    theta3 = PI / 2 - theta3;

    P_d11 << sin(theta1), 0, cos(theta1);

    P_d1 = P_e_1 + L1 * P_d11;
    P_d22 << cos(2 * PI / 3) * sin(theta2), sin(2 * PI / 3)* sin(theta2), cos(theta2);
    P_d2 = P_e_2 + L1 * P_d22;
    P_d33 << cos(4 * PI / 3) * sin(theta3), sin(4 * PI / 3)* sin(theta3), cos(theta3);
    P_d3 = P_e_3 + L1 * P_d33;

    On = (P_d1 + P_d2) / 2;
    v1 = (P_d2 - P_d1) / (P_d2 - P_d1).norm();
    Zn = v1;
    v2 = (P_d3 - P_d2) / (P_d3 - P_d2).norm();
    v3 = v1.cross(v2) / (v1.cross(v2)).norm();
    Yn = v3;
    Xn = v3.cross(v1);

    R12 = sqrt(L2*L2 - (P_d1 - P_d2).norm() * (P_d1 - P_d2).norm() / 4);

    D3E_temp = P_d3 - On;
    D3E = D3E_temp.cross(Xn).norm()/Xn.norm();

    //D3E = v1.dot((P_d2 - P_d1) / 2 + P_d3 - P_d2);
    R3 = sqrt(L2 *L2 - D3E *D3E);
    x3 = abs((P_d3 - P_d2).dot(Xn));

    x_hat = (R12 *R12 - R3 *R3 + x3 * x3) / (2 * x3);
    y_hat = sqrt(R12 * R12 - x_hat * x_hat);

    P_n << x_hat, y_hat, 0, 1;
    T << Xn, Yn, Zn, On,
          0,  0,  0,  1;
    P_b = T * P_n;

    p << P_b(0), P_b(1), P_b(2);

    return p;
}

void Impedance_Control_Delta::Cal_theta3()
{

    b11 = cos(pi1) * e_x + sin(pi1) * e_y + (r_b - r_e);
    b12 = cos(pi2) * e_x + sin(pi2) * e_y + (r_b - r_e);
    b13 = cos(pi3) * e_x + sin(pi3) * e_y + (r_b - r_e);

    b21 = -sin(pi1) * e_x + cos(pi1) * e_y;
    b22 = -sin(pi2) * e_x + cos(pi2) * e_y;
    b23 = -sin(pi3) * e_x + cos(pi3) * e_y;

    b31 = e_z;
    b32 = e_z;
    b33 = e_z;

    theta31 = acos(b21 / L2);
    theta32 = acos(b22 / L2);
    theta33 = acos(b23 / L2);
    theta3 << theta31,
              theta32,
              theta33;

}

void Impedance_Control_Delta::Cal_theta2()
{
    k1 = (b11 *b11 + b21 * b21 + b31 * b31 - L1 * L1 - L2 * L2) / (2 * L1 * L2 * sin(theta31));
    k2 = (b12 *b12 + b22 * b22 + b32 * b32 - L1 * L1 - L2 * L2) / (2 * L1 * L2 * sin(theta32));
    k3 = (b13 *b13 + b23 * b23 + b33 * b33 - L1 * L1 - L2 * L2) / (2 * L1 * L2 * sin(theta33));

    theta21 = acos(k1);
    theta22 = acos(k2);
    theta23 = acos(k3);
    theta2 <<  theta21,
               theta22,
               theta23;

    g11 = L1 + L2 * cos(theta21) * sin(theta31);
    g12 = L1 + L2 * cos(theta22) * sin(theta32);
    g13 = L1 + L2 * cos(theta23) * sin(theta33);

    g21 = L2 * sin(theta21) * sin(theta31);
    g22 = L2 * sin(theta22) * sin(theta32);
    g23 = L2 * sin(theta23) * sin(theta33);

}

void Impedance_Control_Delta::Cal_theta1_d()
{
   /* b11_d = cos(pi1) * p_d(0) + sin(pi1) * p_d(1) + (r_b - r_e);
    b12_d = cos(pi2) * p_d(0) + sin(pi2) * p_d(1) + (r_b - r_e);
    b13_d = cos(pi3) * p_d(0) + sin(pi3) * p_d(1) + (r_b - r_e);

    b21_d = -sin(pi1) * p_d(0) + cos(pi1) * p_d(1);
    b22_d = -sin(pi2) * p_d(0) + cos(pi2) * p_d(1);
    b23_d = -sin(pi3) * p_d(0) + cos(pi3) * p_d(1);

    b31_d = p_d(2);
    b32_d = p_d(2);
    b33_d = p_d(2);

    theta31_d = acos(b21_d / L2);
    theta32_d = acos(b22_d / L2);
    theta33_d = acos(b23_d / L2);

    k1_d = (b11_d *b11_d + b21_d * b21_d + b31_d * b31_d - L1 * L1 - L2 * L2) / (2 * L1 * L2 * sin(theta31_d));
    k2_d = (b12_d *b12_d + b22_d * b22_d + b32_d * b32_d - L1 * L1 - L2 * L2) / (2 * L1 * L2 * sin(theta32_d));
    k3_d = (b13_d *b13_d + b23_d * b23_d + b33_d * b33_d - L1 * L1 - L2 * L2) / (2 * L1 * L2 * sin(theta33_d));

    theta21_d = acos(k1_d);
    theta22_d = acos(k2_d);
    theta23_d = acos(k3_d);

    g11_d = L1 + L2 * cos(theta21_d) * sin(theta31_d);
    g12_d = L1 + L2 * cos(theta22_d) * sin(theta32_d);
    g13_d = L1 + L2 * cos(theta23_d) * sin(theta33_d);

    g21_d = L2 * sin(theta21_d) * sin(theta31_d);
    g22_d = L2 * sin(theta22_d) * sin(theta32_d);
    g23_d = L2 * sin(theta23_d) * sin(theta33_d);

    theta11_d = atan2(-g21_d * b11_d + g11_d * b31_d, g11_d * b11_d + g21_d*  b31_d);
    theta12_d = atan2(-g22_d * b12_d + g12_d * b32_d, g12_d * b12_d + g22_d * b32_d);
    theta13_d = atan2(-g23_d * b13_d + g13_d * b33_d, g13_d * b13_d + g23_d * b33_d);

    theta1_d << theta11_d, theta12_d, theta13_d;*/

}

void Impedance_Control_Delta::Cal_p_i_dot()
{
    w11 = theta1_dot(0) * ww;
    w12 = theta1_dot(1) * ww;
    w13 = theta1_dot(2) * ww;
    w21 = (theta1_dot(0) + theta2_dot(0)) * ww;
    w22 = (theta1_dot(1) + theta2_dot(1)) * ww;
    w23 = (theta1_dot(2) + theta2_dot(2)) * ww;

    ww31(0) = sin(theta11 + theta21);
    ww31(1) = 0;
    ww31(2) = cos(theta11 + theta21);
    w31 = theta3_dot(0) * ww31;
    ww32(0) = sin(theta12 + theta22);
    ww32(1) = 0;
    ww32(2) = cos(theta12 + theta22);
    w32 = theta3_dot(1) * ww32;
    ww33(0) = sin(theta13 + theta23);
    ww33(1) = 0;
    ww33(2) = cos(theta13 + theta23);
    w33 = theta3_dot(2) * ww33;

    u11 << cos(theta11), 0, sin(theta11);
    u12 << cos(theta12), 0, sin(theta12);
    u13 << cos(theta13), 0, sin(theta13);
    u21 << sin(theta31) * cos(theta11 + theta21), cos(theta31), sin(theta31) * sin(theta11 + theta21);
    u22 << sin(theta32) * cos(theta12 + theta22), cos(theta32), sin(theta32) * sin(theta12 + theta22);
    u23 << sin(theta33) * cos(theta13 + theta23), cos(theta33), sin(theta33) * sin(theta13 + theta23);

    p_1_dot = L1 * w11.cross(u11) + L2 * w21.cross(u21) + L2 * w31.cross(u21);
    p_2_dot = L1 * w12.cross(u12) + L2 * w22.cross(u22) + L2 * w32.cross(u22);
    p_3_dot = L1 * w13.cross(u13) + L2 * w23.cross(u23) + L2 * w33.cross(u23);
}

void Impedance_Control_Delta::Cal_Jacobian()
{
    j_q << sin(theta21) * sin(theta31), sin(theta22) * sin(theta32), sin(theta23) * sin(theta33);
    J_q << L1 * j_q(0), 0, 0,
            0, L1* j_q(1), 0,
            0, 0, L1* j_q(2);
    //cout << "J_q : " << J_q << endl;

    j1x = cos(theta11 + theta21) * sin(theta31) * cos(pi1) - cos(theta31) * sin(pi1);
    j2x = cos(theta12 + theta22) * sin(theta32) * cos(pi2) - cos(theta32) * sin(pi2);
    j3x = cos(theta13 + theta23) * sin(theta33) * cos(pi3) - cos(theta33) * sin(pi3);

    j1y = cos(theta11 + theta21) * sin(theta31) * sin(pi1) + cos(theta31) * cos(pi1);
    j2y = cos(theta12 + theta22) * sin(theta32) * sin(pi2) + cos(theta32) * cos(pi2);
    j3y = cos(theta13 + theta23) * sin(theta33) * sin(pi3) + cos(theta33) * cos(pi3);

    j1z = sin(theta11 + theta21) * sin(theta31);
    j2z = sin(theta12 + theta22) * sin(theta32);
    j3z = sin(theta13 + theta23) * sin(theta33);

    J_x <<j1x ,j1y ,j1z,
          j2x, j2y, j2z,
          j3x, j3y, j3z;
    //cout << "J_x : " << J_x << endl;

    //J = J_q.inverse() * J_x;

    p_pdi << p(0)-P_d1(0),p(1)-P_d1(1),p(2)-P_d1(2),
             p(0)-P_d2(0),p(1)-P_d2(1),p(2)-P_d2(2),
             p(0)-P_d3(0),p(1)-P_d3(1),p(2)-P_d3(2);

    p_theta << cos(PI/2-theta11),        -cos(PI/2-theta12)/2,          -cos(PI/2-theta13)/2,
                               0, sqrt(3)*cos(PI/2-theta12)/2,  -sqrt(3)*cos(PI/2-theta13)/2,
              -sin(PI/2-theta11),          -sin(PI/2-theta12),            -sin(PI/2-theta13);

    p_theta = L1*p_theta;

    D_temp = p_pdi*p_theta;

    D.diagonal() << D_temp(0,0) ,D_temp(1,1), D_temp(2,2);

    J = p_pdi.inverse()*D;

    ///cout << "J : " << J << endl;
}

void Impedance_Control_Delta::Cal_theta_dot()
{
    theta1_dot = J * p_dot;

    theta3_dot << -p_1_dot(1) / (L2 * sin(theta31)), -p_2_dot(1) / (L2 * sin(theta32)), -p_3_dot(1) / (L2 * sin(theta33));

    theta2_dot << (-p_1_dot(0) * cos(theta11) - p_1_dot(2) * sin(theta11) + L2 * cos(theta21) * cos(theta31) * theta3_dot(0)) / (L2 * sin(theta21) * sin(theta31)) - theta1_dot(0),
                  (-p_2_dot(0) * cos(theta12) - p_2_dot(2) * sin(theta12) + L2 * cos(theta22) * cos(theta32) * theta3_dot(1)) / (L2 * sin(theta22) * sin(theta32)) - theta1_dot(1),
                  (-p_3_dot(0) * cos(theta13) - p_3_dot(2) * sin(theta13) + L2 * cos(theta23) * cos(theta33) * theta3_dot(2)) / (L2 * sin(theta23) * sin(theta33)) - theta1_dot(2);
}

void Impedance_Control_Delta::Cal_Jacobian_dot()
{
    j1x_dot = -sin(theta11 + theta21) * (theta1_dot(0) + theta2_dot(0)) * sin(theta31) * cos(pi1) + cos(theta11 + theta21) * theta3_dot(0) * cos(theta31) * cos(pi1) + theta3_dot(0) * sin(theta31) * sin(pi1);
    j2x_dot = -sin(theta12 + theta22) * (theta1_dot(1) + theta2_dot(1)) * sin(theta32) * cos(pi2) + cos(theta12 + theta22) * theta3_dot(1) * cos(theta32) * cos(pi2) + theta3_dot(1) * sin(theta32) * sin(pi2);
    j3x_dot = -sin(theta13 + theta23) * (theta1_dot(2) + theta2_dot(2)) * sin(theta33) * cos(pi3) + cos(theta13 + theta23) * theta3_dot(2) * cos(theta33) * cos(pi3) + theta3_dot(2) * sin(theta33) * sin(pi3);

    j1y_dot = -sin(theta11 + theta21) * (theta1_dot(0) + theta2_dot(0)) * sin(theta31) * sin(pi1) + cos(theta11 + theta21) * theta3_dot(0) * cos(theta31) * sin(pi1) - theta3_dot(0) * sin(theta31) * cos(pi1);
    j2y_dot = -sin(theta12 + theta22) * (theta1_dot(1) + theta2_dot(1)) * sin(theta32) * sin(pi2) + cos(theta12 + theta22) * theta3_dot(1) * cos(theta32) * sin(pi2) - theta3_dot(1) * sin(theta32) * cos(pi2);
    j3y_dot = -sin(theta13 + theta23) * (theta1_dot(2) + theta2_dot(2)) * sin(theta33) * sin(pi3) + cos(theta13 + theta23) * theta3_dot(2) * cos(theta33) * sin(pi3) - theta3_dot(2) * sin(theta33) * cos(pi3);

    j1z_dot = cos(theta11 + theta21) * (theta1_dot(0) + theta2_dot(0)) * sin(theta31) + sin(theta11 + theta21) * theta3_dot(0) * cos(theta31);
    j2z_dot = cos(theta12 + theta22) * (theta1_dot(1) + theta2_dot(1)) * sin(theta32) + sin(theta12 + theta22) * theta3_dot(1) * cos(theta32);
    j3z_dot = cos(theta13 + theta23) * (theta1_dot(2) + theta2_dot(2)) * sin(theta33) + sin(theta13 + theta23) * theta3_dot(2) * cos(theta33);

    J_xdot << j1x_dot, j1y_dot, j1z_dot,
              j2x_dot, j2y_dot, j2z_dot,
              j3x_dot, j3y_dot, j3z_dot;

    //cout << "J_xdot : " << J_xdot << endl;

    jq_dot << theta2_dot(0) * cos(theta21) * sin(theta31) + theta3_dot(0) * sin(theta21) * cos(theta31), theta2_dot(1) * cos(theta22) * sin(theta32) + theta3_dot(1) * sin(theta22) * cos(theta32), theta2_dot(2) * cos(theta23) * sin(theta33) + theta3_dot(2) * sin(theta23) * cos(theta33);
   // DiagonalMatrix<double, 3> Jq_dot(L1 * jq_dot);
    Jq_dot << jq_dot(0), 0, 0,
              0, jq_dot(1), 0,
              0, 0, jq_dot(2);

    //cout << "Jq_dot : " << Jq_dot << endl;

}

Vector3d Impedance_Control_Delta::Filtering_loadcell(double loadcell1, double loadcell2, double loadcell3)
{
    double alpha = 0.6;
    // tau_ext => tau_load

   /* loadcell_array[n-1] = tau_ext*3;

    for(int i = 0; i < n-1 ; i ++)
    {
        loadcell_array[i] = loadcell_array[i+1];
    }
    tau_ext = LPF(loadcell_array);*/


    tau_ext_LPF  = alpha*tau_ext_temp + (1-alpha)*tau_ext_LPF;
    tau_ext_temp = tau_ext_LPF;

    return tau_ext_temp;


}

void Impedance_Control_Delta::Trajectory()
{
    //int t3 =0;
    t3 = t3 + 2*PI*0.001;
    //x_traj = 0.1*sin(t3)*cos(10*t3);
    //y_traj = 0.1*sin(t3)*sin(10*t3);
    x_traj = 0.05*cos(t3);
    y_traj = 0.05*sin(t3);
    z_traj = 0.4;//+ 0.15*cos(t3);

    p_traj << x_traj, y_traj, z_traj;
}

void Impedance_Control_Delta::Gravity_Compensation()
{
    //G_comp = -(m1*l1c + m2*l1)*g*v_1 + mp*J.transpose().inverse()*v_3;
}

Vector3d Impedance_Control_Delta::LPF_Position(double cutoff_frequency, double dt, Vector3d  x_d_temp_ , Vector3d x_d_before_)
{
    double rc = 1/(2*PI*cutoff_frequency);
    //cout << "rc : " << rc << endl;
    double alpha = dt / (rc + dt);
    //cout << "alpha : " << alpha << endl;


    Vector3d output = alpha*x_d_temp_ + (1-alpha)*x_d_before_;
    return output;



}
 Vector3d Impedance_Control_Delta :: LPF_Force(double cutoff_frequency, double dt, Vector3d  Force_ , Vector3d force_before)
 {
     double rc = 1/(2*PI*cutoff_frequency);
     //cout << "rc : " << rc << endl;
     double alpha = dt / (rc + dt);
     //cout << "alpha : " << alpha << endl;

     Vector3d output = alpha*Force_ + (1-alpha)*force_before;
     //cout << "output : " << output << endl;
     return output;
 }
