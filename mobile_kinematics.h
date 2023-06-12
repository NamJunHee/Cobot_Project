#ifndef MOBILE_KINEMATICS_H
#define MOBILE_KINEMATICS_H

#include <stdio.h>
#include <iostream>
#include <time.h>
#include <Eigen/Dense>
#include <cmath>
#include <QtMath>

#define PI 3.141592

class Mobile_Kinematics
{
public:
    Mobile_Kinematics();

public:
    void mobile_inverse_kinematic(double x, double y, double w);

    double MotorA_Rpm,MotorB_Rpm,MotorC_Rpm,MotorD_Rpm;

    struct Motor_Angle_Velocity
    {
        float w1;
        float w2;
        float w3;
        float w4;
    };

    double wheel_radius;
};

#endif // MOBILE_KINEMATICS_H
