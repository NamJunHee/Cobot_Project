#include "impedance_control.h"



Impedance_Control::Impedance_Control()
{
    g = 9.8148;
    m = 0.282;
    L_m = 0.059;
    L_load = 0.181;
    L = 0.2;
    //I = (M * (L * L)) / 3;;

    //init_q = 30 * (PI / 180);
    init_dq = 0;
    //30 * (PI / 180);
    dq = 0;
    ddq = 0;

    t = 0;

    F_ext = MatrixXd(3, 1);
    ddx_d = MatrixXd(3,1);
    dx_d = MatrixXd(3, 1);
    x_d = MatrixXd(3, 1);

    q_d = 0;
    dq_d = 0;
    ddq_d = 0;

    q_d_before = 0;
    dq_d_before = 0;
    ddq_d_before = 0;

    K = 0;
    B = 0.0001;

    M = 0.016;
    H = 0;

    m_load = 2;

    J = MatrixXd(3,1);
    ddx_d_before = MatrixXd(3,1);
    dx_d_before = MatrixXd(3,1);
    x_d_before = MatrixXd(3,1);
    x = MatrixXd(3,1);

    tau_ext_LPF = 0;
    cnt = 0;
    i = 1;
  //  cout << "cnt " << cnt << endl;

    time_cnt = 0;
}
double Impedance_Control::LPF(double *input_array)
{
    double output_array[10] ={0,};
    output_array[0] = input_array[0];
    double alpha = 0.8;

    for(int i = 1; i < 10 ; i++)
    {
        output_array[i]  = alpha*input_array[i-1] + (1-alpha)*output_array[i-1];

    }
    return output_array[9];
}


double Impedance_Control::Maf(double *input_array, int n)
{
    double sum = 0;

    for(int i =0; i < n; i++)
    {
           sum += input_array[i];
    }

//    average = sum / n;
/*
    for(int j = 0; j <n; j++)
    {
        if(input_array[j] < average * 0.9 && average * 1.1 < input_array[j])
        {
            output_array[j] = average;
        }
        else
        {
            output_array[j] = input_array[j];
        }
    }
*/
    return sum/n;
}
double Impedance_Control::Torque_calculator(double tau_ext, double q)
{
    double alpha = 0.6;
    // tau_ext => tau_load

   /* loadcell_array[n-1] = tau_ext*3;

    for(int i = 0; i < n-1 ; i ++)
    {
        loadcell_array[i] = loadcell_array[i+1];
    }
    tau_ext = LPF(loadcell_array);*/

    if(tau_ext > 0.1 || tau_ext < -0.1){

        tau_ext_temp = tau_ext;
        F_flag = 1;
    }
    else
    {
        tau_ext_temp = 0;
        F_flag = 0;
    }

    tau_ext_LPF  = alpha*tau_ext_temp + (1-alpha)*tau_ext_LPF;
    tau_ext_temp = tau_ext_LPF;
    /*
    if(tau_ext < 0.1 && tau_ext >-0.1)
    {
        tau_ext =0;
    }
    else
    {
        tau_ext = tau_ext*3;
    }*/

    tau_ext_temp = tau_ext_temp*2;

    t = 0.01;

    int t1 = 0.1;

    if(tau_ext_temp == 0)
    {
        dx_d_before(0,0) = 0;
        dx_d_before(1,0) = 0;
        dx_d_before(2,0) = 0;

        x_d_before(0,0) = 0;
        x_d_before(1,0) = 0;
        x_d_before(2,0) = 0;
    }


    G = m * g * (L_m / 2) * cos(q);

    G_load = m_load * g * L_load * cos(q);

    J(0, 0) = -L * sin(q);
    J(1, 0) = 0;
    J(2, 0) = L * cos(q);

    F_ext = -J * tau_ext_temp;

    //추가 코드

    dx_d_before = dx_d;

    if(tau_ext_temp != 0)
    {
        time_cnt += 0.05;
        dx_d = F_ext/m*exp(-B/m*time_cnt);
        //cout << "time_cnt : "<<time_cnt << endl;
        //cout << "dx_d : "<<dx_d << endl;
    }
    else
    {
        time_cnt =0;
        dx_d << 0,0,0;
        //cout << "dx_d : "<<dx_d << endl;
    }


    ddx_d =(dx_d - dx_d_before)/t;
    ddx_d_before = ddx_d;

    x_d = (dx_d) *t;

    x << L*cos(q),0,L*sin(q);

    x = x + x_d;

    /*if(tau_ext > 0)
    {
        q_d = - atan2(x(0),x(2));
    }
    else
    {
        q_d = atan2(x(0),x(2));
    }*/
    q_d = atan2(x(2),x(0));
    //q_d = q_d * 1.1;
    //if(abs(q_d)<0.0001){q_d = 0;}


    //여기 까지


/*
    dx_d = dx_d_before + ddx_d * t;


    x_d =  x_d_before + ddx_d * (t * t);
    x_d_before = x_d;

    x_d =  dx_d * t;


    if(tau_load > 0)     tau_ext_temp = tau_ext_temp*3;

    {
        q_d = -atan2(F_ext.norm(), L)/4;
    }
    else
    {
        q_d = atan2(F_ext.norm(), L)/4;
    }
    if(abs(q_d)<0.000001){q_d = 0;}

    dq_d = (q_d)/t; //- q_d_before) / t;
    if(abs(dq_d)<0.000001){dq_d = 0;}

    if(tau_ext ==0){dq = 0;}
    else {dq = (q - q_before)/t;}

    q_before = q;

    ddq_d = (dq_d)/t; //- dq_d_before) / t;
    if(abs(ddq_d)<0.000001){ddq_d = 0;}
    dq_d_before = dq_d;

    if((int)dq == (int)dq_before)
    {
        ddq = 0;
        //cout <<"ddq : " << ddq << endl;
    }
    else
    {
        ddq = (dq - dq_before)/t;
        //cout <<"ddq : " << ddq << endl;
    }

    //ddq1 = (dq - dq_before)/t1;
    */



    tau_gravity = G+ G_load;

    tau1 = M* ddq + tau_gravity;

    tau2 = M*ddq + G;

    tau  = K * (q_d - q) + B * (dq_d - dq) + M * ddq_d + tau_gravity*0.85; //+ G;

    tau_ext1 = M *(ddq_d-ddq) + B *(dq_d -dq) + K*(q_d-q);
    err_vel = B * ( dq_d - dq);

    q_d_before = q_d;
    dq_d_before = dq_d;
    ddq_d_before = ddq_d;
    dq_before = dq;

    if (tau*1000 <50 && tau*1000 > 0) {tau = 0;}
    else if(tau*1000< 0 && tau*1000 > -50){tau = 0;}

    input_array[n-1] = tau;

    for(int i = 0; i < n-1 ; i ++)
    {
        input_array[i] = input_array[i+1];
    }
    tau = Maf(input_array,n);

    if (tau < -4.0)
    {
//        cout <<"dq_before : " << dq_before << endl;
//        cout <<"dq : " << dq << endl;
//        cout <<"dq-dq_before : " << dq-dq_before << endl;
//        cout <<"ddq : " << ddq << endl;
//        cout << "tau_gravity : " << tau_gravity << endl;
//        cout << "tau1 : " << tau << endl;
        tau = -4.0;
    }
    else if (tau > 4.0)
    {
//        cout <<"dq_before : " << dq_before << endl;
//        cout <<"dq : " << dq << endl;
//        cout <<"dq-dq_before : " << dq-dq_before << endl;
//        cout <<"ddq : " << ddq << endl;
//        cout << "tau_gravity : " << tau_gravity << endl;
//        cout << "tau1 : " << tau1 << endl;
        tau = 4.0;
    }

    tau_ext1 = tau_ext_temp;
    return tau * 1400;
}

