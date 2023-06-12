    #include "mobile_kinematics.h"

Mobile_Kinematics::Mobile_Kinematics()
{
    wheel_radius = 0.076;
}

void Mobile_Kinematics::mobile_inverse_kinematic(double x, double y, double w)
{

    double link1 = 0.193;//193mm
    double link2 = 0.089;//89mm

    double inverse_array[4][3] = { {1,1,-(link1 + link2)},
            {1,-1,+(link1 + link2)},
            {1,-1,-(link1 + link2)},
            {1,1,+(link1 + link2)} };

    if(x<0) {
        x=x*2;
    }
    else{
        x=x;
    }
    double velocity_array[3][1] = { {x},
            {-y},
            {w} };

    double wheel_omeaga_array[4][1] = { 0 };
    int i,j,p = 0;

    for ( i = 0; i < 4; i++) {
        for ( j = 0; j < 1; j++) {
            for( p=0;p<3;p++){
                wheel_omeaga_array[i][j] += inverse_array[i][p] * velocity_array[p][j];
            }
        }
    }

    Motor_Angle_Velocity motor_omega;
    //rad/s
    //INPUT m/s
    // 0.01 output cm/s
    motor_omega.w1=((1.0/wheel_radius)*wheel_omeaga_array[0][0])*0.01;
    motor_omega.w2=((1.0/wheel_radius)*wheel_omeaga_array[1][0])*0.01;
    motor_omega.w3=((1.0/wheel_radius)*wheel_omeaga_array[2][0])*0.01;
    motor_omega.w4=((1.0/wheel_radius)*wheel_omeaga_array[3][0])*0.01;

    //rpm = (rad/s) * (60 / (2?))
    MotorA_Rpm = (60/ (2*PI)) * motor_omega.w1;
    MotorB_Rpm = (60/ (2*PI)) * motor_omega.w2;
    MotorC_Rpm = (60/ (2*PI)) * motor_omega.w3;
    MotorD_Rpm = (60/ (2*PI)) * motor_omega.w4;

}
