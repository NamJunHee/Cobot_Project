#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "iostream"

using namespace std;

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    Dserial = new QSerialPort(this);
    Mserial = new QSerialPort(this);
    IMUSerial = new QSerialPort(this);

    QList<QSerialPortInfo> list;
    list = QSerialPortInfo::availablePorts();

    _5ms_Timer = new QTimer(this);
    connect(_5ms_Timer, SIGNAL(timeout()),this,SLOT(calRun()));
    _5ms_Timer->start(4);

    _10ms_Timer = new QTimer(this);
    connect(_10ms_Timer, SIGNAL(timeout()),this,SLOT(serialSend()));
    _10ms_Timer->start(4);

    _20ms_Timer = new QTimer(this);
    connect(_20ms_Timer, SIGNAL(timeout()),this,SLOT(update()));
    _20ms_Timer->start(20);

    _100ms_Timer = new QTimer(this);
    //connect(_100ms_Timer, SIGNAL(timeout()), this, SLOT(IMUCallback()));
    _100ms_Timer->start(10);

    delay_time = new QTimer(this);

    connect(IMUSerial,SIGNAL(readyRead()),this,SLOT(IMUCallback()));
    connect(Dserial,SIGNAL(readyRead()),this,SLOT(serialReceived()));
    connect(Mserial,SIGNAL(readyRead()),this,SLOT(serialReceived()));
}

MainWindow::~MainWindow()
{
    delete ui;
    delete _20ms_Timer;
    delete _10ms_Timer;
    delete _5ms_Timer;
    delete _50ms_Timer;
    delete _1ms_Timer;
    delete _100ms_Timer;
    delete _2ms_Timer;

    if(Dserial->isOpen()){
        Dserial->close();
    }
    if(Mserial->isOpen()){
        Mserial->close();
    }
    if(IMUSerial->isOpen()){
        IMUSerial->close();
    }
}
void MainWindow::keyPressEvent(QKeyEvent *event)
{
    if(event->key()==Qt::Key_W||event->key()==Qt::Key_Return){
        mode_flag = 1;
        imp.mode_switch  = IK_mode;
        imp.p_inv(0) = imp.p_inv(0) + 0.05;
        qDebug() << "W pressed" << "\n";
    }
    else if(event->key()==Qt::Key_S||event->key()==Qt::Key_Return){
        mode_flag = 1;
        imp.mode_switch  = IK_mode;
        imp.p_inv(0) = imp.p_inv(0) - 0.05;
        qDebug() << "S pressed" << "\n";
    }
    else if(event->key()==Qt::Key_A||event->key()==Qt::Key_Return){
        mode_flag = 1;
        imp.mode_switch  = IK_mode;
        imp.p_inv(1) = imp.p_inv(1) - 0.05;
        qDebug() << "A pressed" << "\n";
    }
    else if(event->key()==Qt::Key_D||event->key()==Qt::Key_Return){
        mode_flag = 1;
        imp.mode_switch  = IK_mode;
        imp.p_inv(1) = imp.p_inv(1) + 0.015;
        qDebug() << "D pressed" << "\n";
    }
    else if(event->key()==Qt::Key_Q||event->key()==Qt::Key_Return){
        mode_flag = 1;
        imp.mode_switch  = IK_mode;
        imp.p_inv(2) = imp.p_inv(2) - 0.05;
        qDebug() << "Q pressed" << "\n";
    }
    else if(event->key()==Qt::Key_E||event->key()==Qt::Key_Return){
        mode_flag = 1;
        imp.mode_switch  = IK_mode;
        imp.p_inv(2) = imp.p_inv(2) + 0.05;
        qDebug() << "E pressed" << "\n";
    }
}


//--------------------Delta_Cal_function------------------//
void MainWindow::calRun()
{
    if(Connect_Sign ==1)
    {
        TorqueOut = imp.Torque_Cal_Delta(LoadCell1, LoadCell2, LoadCell3 ,qDegreesToRadians(270-Encoder1),qDegreesToRadians(270-Encoder2),qDegreesToRadians(270-Encoder3));

        if(SW_Mode == Transport_mode)
        {
            if((imp.F_ext_mobile(0) > m_limit || imp.F_ext_mobile(0) < -m_limit ) && (imp.F_ext_mobile(1) > -m_limit && imp.F_ext_mobile(1) < m_limit))
            {
                imp.F_ext_mobile(1) = 0;
            }
            else if((imp.F_ext_mobile(1) > m_limit || imp.F_ext_mobile(1) < -m_limit ) && (imp.F_ext_mobile(0) > -m_limit && imp.F_ext_mobile(0) < m_limit))
            {
                imp.F_ext_mobile(0) = 0;
            }
            else if((imp.F_ext_mobile(0) < m_limit && imp.F_ext_mobile(0) > -m_limit ) && (imp.F_ext_mobile(1) < m_limit && imp.F_ext_mobile(1) > -m_limit ))
            {
                imp.F_ext_mobile(0) = 0;
                imp.F_ext_mobile(1) = 0;
            }

            mk.mobile_inverse_kinematic(imp.F_ext_mobile(0),imp.F_ext_mobile(1),0);
        }
    }
    if(write_tau_flag == 1)
    {
        write(tau_Txt);
    }

    if(write_pos_flag == 1)
    {
        write(pos_Txt);
    }
}
void MainWindow::Positon_Init()
{
    mode_flag = 1;
    imp.mode_switch  = IK_mode;
    QString x,y,z;

    imp.p_inv(0) = 0;
    imp.p_inv(1) = 0;
    imp.p_inv(2) = 0.35;

    ui->inv_x->setText(QString::number(imp.p_inv(0)));
    ui->inv_y->setText(QString::number(imp.p_inv(1)));
    ui->inv_z->setText(QString::number(imp.p_inv(2)));
}
//----------------------IMU Function----------------------//
void MainWindow::IMU_init()
{
    TxBuffer[0] = A;
    TxBuffer[1] = N;
    TxBuffer[2] = G;
    TxBuffer[3] = CR;
    TxBuffer[4] = LF;
    TxBuffer[5] = G;
    TxBuffer[6] = Y;
    TxBuffer[7] = R;
    TxBuffer[8] = CR;
    TxBuffer[9] = LF;

    for(int i = 0; i < sizeof(TxBuffer); i++)
    {
        Tx_data.push_back(TxBuffer[i]);
    }

    IMUSerial->write(Tx_data,sizeof(TxBuffer));
}

void MainWindow::IMUCallback()
{
    IMUSerial->write(Tx_data,10);

    if(IMUSerial->isReadable())
    {
        Rx_data = IMUSerial->readAll();
        // qDebug() <<"Rx_data" << Rx_data << endl;
        // qDebug() <<"size Data" << Rx_data.length() << endl;

        int i = 0;

        for(int j = 0; j < sort_count; j++)
        {
            if(Rx_data[j] == 'a')
            {
                a_flag = 1;
                a_array = j;
            }
        }

        if(a_flag == 1)
        {
            for(int k = 0; k < sort_count; k++)
            {
                if(a_array + k > sort_count)
                {
                    Rx_sort_data[k] = Rx_data[(a_array + k) - sort_count];
                }
                else
                {
                    Rx_sort_data[k] = Rx_data[a_array + k];
                }
            }
        }

        a_flag = 0;

        char *Rx_Buf = Rx_sort_data.data();
        qDebug() <<"Rx_sort_data" << Rx_sort_data << endl;

        char * ptr;
        ptr = strtok(Rx_Buf, " \n");

        if(Rx_sort_data[0] == 'a')
        {
            while(ptr != NULL)
            {
                if(i == 1)
                    imu.roll = atof(ptr);
                else if(i == 2)
                    imu.pitch = atof(ptr);
                else if(i == 3)
                    imu.yaw = atof(ptr);
                else if(i == 5)
                    imu.roll_vel = atof(ptr);
                else if(i == 6)
                    imu.pitch_vel = atof(ptr);
                else if(i == 7)
                    imu.yaw_vel = atof(ptr);

                ptr = strtok(NULL, " \n");
                i++;
            }

            if(imu.yaw < 0)
                imu.yaw += 360;
            else if(imu.yaw > 360)
                imu.yaw = 360;

            IMUSerial->write(Tx_data,10);
            Rx_data.clear();


        }

        //        cout << "roll = " << imu.roll << endl;
        //        cout << "pitch = " << imu.pitch << endl;
        //        cout << "yaw = " << imu.yaw << endl;
        //        cout << "roll_vel = " << imu.roll_vel << endl;
        //        cout << "pitch_vel = " << imu.pitch_vel << endl;
        //        cout << "yaw_vel = " << imu.yaw_vel << endl;
        //        cout << endl;
    }
}

//----------------------Serial Function-------------------//
void MainWindow::serialSend()
{
    if(time>49){
        time = 10;
    }

    if(Connect_Sign == 1){
        time++;
        QByteArray data;
        QString D_Str, M_Str, str1, str2, str3, mtr1, mtr2, mtr3, mat_str;

        D_Str = Send_Angle(imp.theta1_d(0),imp.theta1_d(1),imp.theta1_d(2),imp.F_flag1,imp.F_flag2,imp.F_flag3);//여기 바꿔야함

        if(D_Str.size() == 24)
        {
            Dserial->write(D_Str.toStdString().c_str(), D_Str.size());
        }

        M_Str = Send_RPM(mk.MotorA_Rpm,mk.MotorB_Rpm,mk.MotorC_Rpm,mk.MotorD_Rpm);
        // qDebug() << "M_Str" << M_Str;
        // qDebug() << "M_Str.size" << M_Str.size();

        if(M_Str.size() == 20)
        {
            Mserial->write(M_Str.toStdString().c_str(), M_Str.size());
        }
    }
}

void MainWindow::serialReceived()
{
    if(Connect_Sign == 1){
        //        IMUCallback();

        QString load_Str, encoder_Str;

        load_Str = Mserial->readAll();
        encoder_Str = Dserial->readAll();

        // qDebug() << "load_list:" << load_Str;
        // qDebug() << "Delta_list:" << encoder_Str;

        QStringList T_list1 = load_Str.split("a");
        QStringList T_list2 = load_Str.split("b");
        QStringList T_list3 = load_Str.split("c");
        QStringList T_list4 = load_Str.split("d");

        int Load_check1 = T_list1.size();
        int Load_check2 = T_list2.size();
        int Load_check3 = T_list3.size();
        int Mode_check = T_list4.size();

        //        qDebug() << "Load_check1:" << Load_check1;
        //        qDebug() << "Load_check2:" << Load_check2;
        //        qDebug() << "Load_check3:" << Load_check3;
        //        qDebug() << "Mode_check:" << Mode_check;

        QStringList E_list1 = encoder_Str.split("a");
        QStringList E_list2 = encoder_Str.split("b");
        QStringList E_list3 = encoder_Str.split("c");

        int Encoder_check1 = E_list1.size();
        int Encoder_check2=  E_list2.size();
        int Encoder_check3 = E_list3.size();

        if((Load_check1 > 1) && (Load_check1 < 40))
        {
            QString Loadcell1 = T_list1.at(1);
            Loadcell1 = Loadcell1.remove(3,Loadcell1.length());
            LoadCell1_temp = Loadcell1_Calibration(Loadcell1.remove('@').toFloat());
            LoadCell1 = LoadCell1_temp  - TorqueOut(0);//Loadcell_Calibration(Loadcell1.remove('@').toFloat()/load_var);// - TorqueOut(0);
        }
        if((Load_check2 > 1) && (Load_check2 < 40))
        {
            QString Loadcell2 = T_list2.at(1);
            Loadcell2 = Loadcell2.remove(3,Loadcell2.length());
            LoadCell2_temp = Loadcell2_Calibration(Loadcell2.remove('@').toFloat());
            LoadCell2 = LoadCell2_temp  - TorqueOut(1);//Loadcell_Calibration(Loadcell2.remove('@').toFloat()/load_var);// - TorqueOut(1);
        }
        if((Load_check3 > 1) && (Load_check3 < 40))
        {
            QString Loadcell3 = T_list3.at(1);
            Loadcell3 = Loadcell3.remove(3,Loadcell3.length());
            LoadCell3_temp = Loadcell3_Calibration(Loadcell3.remove('@').toFloat());
            LoadCell3 = LoadCell3_temp  - TorqueOut(2);//Loadcell3_Linealization(load3);// - TorqueOut(2);
        }
        if((Mode_check > 1) && (Mode_check < 40))
        {
            QString Mode = T_list4.at(1);
            Mode = Mode.remove(1,Mode.length());
            //qDebug() << "mode:" << Mode;

            if((Mode.toInt() == 1) || (Mode.toInt() == 2) || (Mode.toInt() == 3))
            {
                SW_Mode = Mode.toInt();
            }

            if(SW_Mode==Transport_mode)
            {
                Positon_Init();
                //imp.mode_switch = Init_mode;
                ui->Mode->setText("[MODE "+Mode+" : Transport Work]");
                G_compensation_flag = 0;
            }
            else if(SW_Mode==2)
            {
                ui->Mode->setText("[MODE "+Mode+" : Control]");
                G_compensation_flag = 0;

            }
            else if(SW_Mode==Precison_mode)
            {
                imp.mode_switch = Impedance_mode;
                mode_flag = 1;

                if(G_compensation_flag == 0)
                {
                    imp.m_est = imp.J.transpose()*imp.gravity_temp;//*G/g;
                    imp.mp = (LoadCell1_temp + LoadCell2_temp + LoadCell3_temp)/(imp.m_est(0)+imp.m_est(1)+imp.m_est(2));
                    G_compensation_flag = 1;
                }

                ui->Mode->setText("[MODE "+Mode+" : Precision Work]");
            }
            else
            {
                ui->Mode->setText("Mode Error");
            }
        }


        if((Encoder_check1 > 2) && (Encoder_check1 < 40))
        {
            QString a = E_list1.at(2);
            a = a.remove(5,a.length());
            if(a.toInt()!=0)
            {
                if(a.toInt()<31000 && a.toInt()>10000)
                {
                    Encoder1 = (a.remove('@').toFloat())/100;
                    //qDebug() << "Encoder1 :" << Encoder1;
                }

            }
        }
        if((Encoder_check2 > 2) && (Encoder_check2 < 40))
        {
            QString b = E_list2.at(2);
            b = b.remove(5,b.length());
            if(b.toInt()!=0)
            {
                if(b.toInt()<31000 && b.toInt()>10000)
                {
                    Encoder2 = (b.remove('@').toFloat())/100;
                    b = b.remove(5,b.length());
                    //qDebug() << "Encoder2 :" << Encoder2;
                }
            }
        }
        if((Encoder_check3 > 2) && (Encoder_check3 < 40))
        {
            QString c = E_list3.at(2);
            c = c.remove(5,c.length());
            if(c.toInt()!=0)
            {
                if(c.toInt()<31000 && c.toInt()>10000)
                {
                    Encoder3 = (c.remove('@').toFloat())/100;
                    //qDebug() << "Encoder3 :" << Encoder3;
                }
            }
        }
    }
}

QString MainWindow::Send_Angle(double q1, double q2, double q3, int F_flag1, int F_flag2,int F_flag3)
{
    QString str1,str2,str3,strF,strMode,D_Str;

    int q_d1 = 270 * 100 - int(qRadiansToDegrees(q1) * 100);
    int q_d2 = 270 * 100 - int(qRadiansToDegrees(q2) * 100);
    int q_d3 = 270 * 100 - int(qRadiansToDegrees(q3) * 100);

    str1 = "A" + QString::number(q_d1);
    str2 = "B" + QString::number(q_d2);
    str3 = "C" + QString::number(q_d3);
    strF = "D" + QString::number(F_flag1) +QString::number(F_flag2) + QString::number(F_flag3);
    strMode = "E" + QString::number(mode_flag);
    D_Str = str1+str2+str3+strF+strMode;

    time_cnt = time_cnt + 0.001;
    //qDebug() << time_cnt << "n";
    //qDebug() << D_Str << endl;
    return D_Str;
}

QString MainWindow::Send_RPM(float w1, float w2, float w3, float w4)
{
    QString A_Rpm,B_Rpm,C_Rpm,D_Rpm, M_Str;

    int A = w1*10;
    int B = w2*10;
    int C = w3*10;
    int D = w4*10;

    if(A > 0)
    {
        if(A < 100 && A > 9){ A_Rpm = "a+" + QString::number(A)+"@"; }
        else if(A < 10){ A_Rpm = "a+" + QString::number(A)+"@@"; }
        else{ A_Rpm = "a+" + QString::number(A); }
    }
    else if(A < 0)
    {
        if(A > -100 && A < -9){ A_Rpm = "a" + QString::number(A)+"@"; }
        else if(A > -10){ A_Rpm = "a" + QString::number(A)+"@@"; }
        else{ A_Rpm = "a" + QString::number(A); }
    }
    else
    {
        A_Rpm = "a" + QString::number(A)+"@@@";
    }

    if(B > 0)
    {
        if(B < 100 && B > 9){ B_Rpm = "b+" + QString::number(B)+"@"; }
        else if(B < 10){ B_Rpm = "b+" + QString::number(B)+"@@"; }
        else{ B_Rpm = "b+" + QString::number(B); }
    }
    else if(B < 0)
    {
        if(B > -100 && B < -9){ B_Rpm = "b" + QString::number(B)+"@"; }
        else if(B > -10){ B_Rpm = "b" + QString::number(B)+"@@"; }
        else{ B_Rpm = "b" + QString::number(B); }
    }
    else
    {
        B_Rpm = "b" + QString::number(B)+"@@@";
    }

    if(C > 0)
    {
        if(C < 100 && C > 9){ C_Rpm = "c+" + QString::number(C)+"@"; }
        else if(C < 10){ C_Rpm = "c+" + QString::number(C)+"@@"; }
        else{ C_Rpm = "c+" + QString::number(C); }
    }
    else if(C < 0)
    {
        if(C > -100 && C < -9){ C_Rpm = "c" + QString::number(C)+"@"; }
        else if(C > -10){ C_Rpm = "c" + QString::number(C)+"@@"; }
        else{ C_Rpm = "c" + QString::number(C); }
    }
    else
    {
        C_Rpm = "c" + QString::number(C)+"@@@";
    }

    if(D > 0)
    {
        if(D < 100 && D > 9){ D_Rpm = "d+" + QString::number(D)+"@"; }
        else if(D < 10){ D_Rpm = "d+" + QString::number(D)+"@@"; }
        else{ D_Rpm = "d+" + QString::number(D); }
    }
    else if(D < 0)
    {
        if(D > -100 && D < -9){ D_Rpm = "d" + QString::number(D)+"@"; }
        else if(D > -10){ D_Rpm = "d" + QString::number(D)+"@@"; }
        else{ D_Rpm = "d" + QString::number(D); }
    }
    else
    {
        D_Rpm = "d" + QString::number(D)+"@@@";
    }

    M_Str = A_Rpm+B_Rpm+C_Rpm+D_Rpm;

    return M_Str;
}

void MainWindow::write(const QString filename)
{
    QFile file(filename);
    if(!file.open(QIODevice::WriteOnly | QIODevice::Text| QIODevice::Append))
    {
        qDebug() << "Could not open"<< filename <<"file for writing";
        return;
    }
    QTextStream out(&file);

    if(write_tau_flag == 1)
    {
        //out << imp.tau_delta(0) << "\t" << imp.tau_delta(1) << "\t" << imp.tau_delta(2) << imp.load1 << "\t" << imp.load2 << "\t" << imp.load3 <<"\n";
        out << imp.F_ext(0) << "\t" << imp.F_ext(1) << "\t" << imp.F_ext(2) <<endl;
    }

    if(write_pos_flag == 1)
    {
        out << imp.p(0) << "\t" << imp.p(1) << "\t" << imp.p(2) <<"\t" << imp.p_d(0) << "\t" << imp.p_d(1) << "\t" << imp.p_d(2) << "\t" <<imp.p_traj(0) << "\t" << imp.p_traj(1) << "\t" << imp.p_traj(2) << "\n";
        //out >> imp.p_traj(0) << "\t" << imp.p_traj(1) << "\t" << imp.p_traj(2) << "\n";
    }

    file.close();

    return;
}

void MainWindow::read (const QString filename)
{

    QFile file(filename);
    if(!file.open(QIODevice::ReadOnly|QIODevice::Text))
    {
        qDebug() << "Could not open file ";
        return;
    }

    QTextStream in(&file);
    QString myText;


    while(!in.atEnd())
    {
        QStringList myList = in.readLine().split('\t');
        //imp.p_traj(0) = QString::number(myList(0));

    }
    qDebug() << myText;

    file.close();
}

//--------------------LoadCell Function------------------//
double MainWindow::Loadcell_Linealization(double x)
{
    double result = 0.0052*x*x*x + 0.0208*x*x + 0.1325*x + 0.0024;

    return result;
}

double MainWindow::Loadcell1_Calibration(double x)
{
    double result = 0;
    result = 0.0617*x - 0.0239;

    return result;
}

double MainWindow::Loadcell2_Calibration(double x)
{
    double result = 0;
    result = 0.0569 * x+0.0776;

    return result;
}

double MainWindow::Loadcell3_Calibration(double x)
{
    double result = 0;
    result = 0.0584*x- 0.0876;

    return result;
}

//--------------------- UI_function-----------------------//
void MainWindow::update()
{
    if(init_flag == 1)
    {
        ui->inv_x->setText(QString::number(imp.p_inv(0)));
        ui->inv_y->setText(QString::number(imp.p_inv(1)));
        ui->inv_z->setText(QString::number(imp.p_inv(2)));

        ui->Mass->setText(QString::number(imp.m));
        ui->Damper->setText(QString::number(imp.B));
        ui->Spring->setText(QString::number(imp.K));
        ui->Cutoff_freq->setText(QString::number(imp.cutoff_frequency));
        ui->k_gain->setText(QString::number(imp.k_gain));
        init_flag = 0 ;
    }

    if(ui_flag == 1)
    {

        //    ui->p_x->setText(QString::number(imp.p(0)));
        //    ui->p_y->setText(QString::number(imp.p(1)));
        //    ui->p_z->setText(QString::number(imp.p(2)));
        ui->Tlc1->setText(QString::number(LoadCell1)+" [Nm]");
        ui->Tlc2->setText(QString::number(LoadCell2)+" [Nm]");
        ui->Tlc3->setText(QString::number(LoadCell3)+" [Nm]");

        ui->Enc3->setText(QString::number(270-Encoder3)+" [Degree]");
        ui->Enc2->setText(QString::number(270-Encoder2)+" [Degree]");
        ui->Enc1->setText(QString::number(270-Encoder1)+" [Degree]");

        if(imu.roll != 0)
            ui->IMU_r->setText(QString::number(imu.roll));

        if(imu.pitch != 0)
            ui->IMU_p->setText(QString::number(imu.pitch));

        if(imu.yaw_vel != 0)
            ui->IMU_y->setText(QString::number(imu.yaw_vel));

        ui->X_Pos->setText(QString::number(imp.F_ext_mobile(0)));
        ui->Y_Pos->setText(QString::number(imp.F_ext_mobile(1)));

        ui->M_FL->setText(QString::number(mk.MotorA_Rpm)+" [RPM]");
        ui->M_FR->setText(QString::number(mk.MotorB_Rpm)+" [RPM]");
        ui->M_BL->setText(QString::number(mk.MotorC_Rpm)+" [RPM]");
        ui->M_BR->setText(QString::number(mk.MotorD_Rpm)+" [RPM]");

        ui->p_d_x->setText(QString::number(imp.p_d(0)));
        ui->p_d_y->setText(QString::number(imp.p_d(1)));
        ui->p_d_z->setText(QString::number(imp.p_d(2)));

        //    ui->F_ext_x->setText(QString::number(imp.F_ext(0)));
        //    ui->F_ext_y->setText(QString::number(imp.F_ext(1)));
        //    ui->F_ext_z->setText(QString::number(imp.F_ext(2)));
        //
        //
        //
        //    ui->theta11_d->setText(QString::number(qRadiansToDegrees(imp.theta1_d(0))));
        //    ui->theta12_d->setText(QString::number(qRadiansToDegrees(imp.theta1_d(1))));
        //    ui->theta13_d->setText(QString::number(qRadiansToDegrees(imp.theta1_d(2))));

        //    ui->Tor_Out1->setText(QString::number(TorqueOut(0))+" [Nm]");
        //    ui->Tor_Out2->setText(QString::number(TorqueOut(1))+" [Nm]");
        //    ui->Tor_Out3->setText(QString::number(TorqueOut(2))+" [Nm]");



        //    ui->F_flag1->setText(QString::number(imp.F_flag1));
        //    ui->F_flag2->setText(QString::number(imp.F_flag2));
        //    ui->F_flag3->setText(QString::number(imp.F_flag3));

        //    ui->dx_d_x->setText(QString::number(imp.p_traj(0)));
        //    ui->dx_d_y->setText(QString::number(imp.p_traj(1)));
        //    ui->dx_d_z->setText(QString::number(imp.p_traj(2)));
        //
        //
        //

        //    ui->M_FR->setText(QString::number(mk.MotorA_Rpm)+" [rad/s]");
        //    ui->M_FL->setText(QString::number(mk.MotorB_Rpm)+" [rad/s]");
        //    ui->M_BR->setText(QString::number(mk.MotorC_Rpm)+" [rad/s]");
        //    ui->M_BL->setText(QString::number(mk.MotorD_Rpm)+" [rad/s]");
        /*
   //theta1_d_out(0)


    ui->x_d_x->setText(QString::number(imp.x_d(0)));
    ui->x_d_y->setText(QString::number(imp.x_d(1)));
    ui->x_d_z->setText(QString::number(imp.x_d(2)));

    ui->F_flag3->setText(QString::number(imp.F_flag3));

    ui->dx_d_x->setText(QString::number(imp.p_traj(0)));
    ui->dx_d_y->setText(QString::number(imp.p_traj(1)));
    ui->dx_d_z->setText(QString::number(imp.p_traj(2)));

    ui->J_x->setText(QString::number(imp.J(0)));
    ui->J_y->setText(QString::number(imp.J(1)));
    ui->J_z->setText(QString::number(imp.J(2)));
    ui->p_dot_x->setText(QString::number(imp.p_dot(0)));
    ui->p_dot_y->setText(QString::number(imp.p_dot(1)));
    ui->p_dot_z->setText(QString::number(imp.p_dot(2)));
    ui->theta11_dot->setText(QString::number(imp.theta1_dot(0)));
    ui->theta12_dot->setText(QString::number(imp.theta1_dot(1)));
    ui->theta13_dot->setText(QString::number(imp.theta1_dot(2)));
    ui->theta11_ddot->setText(QString::number(imp.theta1_ddot(0)));
    ui->theta12_ddot->setText(QString::number(imp.theta1_ddot(1)));
    ui->theta13_ddot->setText(QString::number(imp.theta1_ddot(2)));
*/}
}

void MainWindow::on_serial_port_set_clicked()
{
    Mserial->setPortName("ttyUSB0");
    Mserial->setBaudRate(QSerialPort::Baud115200);
    Mserial->setDataBits(QSerialPort::Data8);
    Mserial->setParity(QSerialPort::NoParity);
    Mserial->setStopBits(QSerialPort::OneStop);
    Mserial->setFlowControl(QSerialPort::NoFlowControl);
    if(Mserial->open(QIODevice::ReadWrite)){
        ui->information->appendPlainText("Mobile Serial Port Connect!!");
    }
    else{
        Mserial->close();
        ui->information->appendPlainText("Mobile Serial Port Open Error");
    }

    Dserial->setPortName("ttyUSB1");
    Dserial->setBaudRate(QSerialPort::Baud115200);
    Dserial->setDataBits(QSerialPort::Data8);
    Dserial->setParity(QSerialPort::NoParity);
    Dserial->setStopBits(QSerialPort::OneStop);
    Dserial->setFlowControl(QSerialPort::NoFlowControl);
    if(Dserial->open(QIODevice::ReadWrite)){
        ui->information->appendPlainText("delta Serial Port Connect!!");
    }
    else{
        Dserial->close();
        ui->information->appendPlainText("Delta Serial Port Open Error");
    }

    IMUSerial->setPortName("ttyUSB2");
    IMUSerial->setBaudRate(QSerialPort::Baud57600);
    IMUSerial->setDataBits(QSerialPort::Data8);
    IMUSerial->setParity(QSerialPort::NoParity);
    IMUSerial->setStopBits(QSerialPort::OneStop);
    IMUSerial->setFlowControl(QSerialPort::NoFlowControl);
    if(IMUSerial->open(QIODevice::ReadWrite)){
        ui->information->appendPlainText("IMU Serial Port Connect!!");
        IMU_init();
    }
    else{
        IMUSerial->close();
        ui->information->appendPlainText("IMUSerial Port Open Error");
    }
}

void MainWindow::on_serial_connect_clicked()
{
    QByteArray data;
    QString D_Str, M_Str, IMU_Str;

    D_Str = "Delta Connect";
    Dserial->write(D_Str.toStdString().c_str(), D_Str.size());
    ui->information->appendPlainText(D_Str);

    M_Str = "Mobile Connect";
    Mserial->write(M_Str.toStdString().c_str(), M_Str.size());
    ui->information->appendPlainText(M_Str);

    IMU_Str = "IMU Connect";
    IMUSerial->write(IMU_Str.toStdString().c_str(), IMU_Str.size());
    ui->information->appendPlainText(IMU_Str);

    Connect_Sign = 1;
    ui->information->appendPlainText("Connect_Sign :"+QString::number(Connect_Sign));
}

void MainWindow::on_serial_disconnect_clicked()
{
    Connect_Sign = 0;
    QByteArray data;
    QString D_Str, M_Str, IMU_Str;

    D_Str = "Delta Connect End!!";
    Dserial->write(D_Str.toStdString().c_str(), D_Str.size());
    Dserial->close();

    ui->information->appendPlainText(D_Str);

    M_Str = "Mobile Connect End!!";
    Mserial->write(M_Str.toStdString().c_str(), M_Str.size());
    Mserial->close();

    ui->information->appendPlainText(M_Str);

    IMU_Str = "IMU Connect End!!";
    IMUSerial->write(IMU_Str.toStdString().c_str(), IMU_Str.size());
    IMUSerial->close();

    ui->information->appendPlainText(IMU_Str);

    ui->information->appendPlainText("Connect_Sign :"+QString::number(Connect_Sign));
}

void MainWindow::on_Trajectory_clicked()
{
    imp.mode_switch = Trajectory_mode;

    mode_flag = 1;
}

void MainWindow::on_impedance_clicked()
{
    imp.mode_switch = Impedance_mode;
    mode_flag = 1;
}

void MainWindow::on_position_clicked()
{
    imp.mode_switch = Init_mode;
    mode_flag = 0;
}

void MainWindow::on_UI_Button_toggled(bool checked)
{
    if(checked == 1)
    {
        ui_flag = 1;
    }
    else
    {
        ui_flag = 0;
    }
}

void MainWindow::on_write_tau_toggled(bool checked)
{
    if(checked == 1)
    {
        write_tau_flag = 1;
    }
    else
    {
        write_tau_flag = 0;
    }
}

void MainWindow::on_write_pos_toggled(bool checked)
{
    if(checked == 1)
    {
        write_pos_flag = 1;
    }
    else
    {
        write_pos_flag = 0;
    }
}

void MainWindow::on_pushButton_3_clicked()
{
    QString M,B,K,Cutoff,k_gain;

    M = ui->Mass->text();
    B = ui->Damper->text();
    K = ui->Spring->text();
    Cutoff = ui->Cutoff_freq->text();
    k_gain = ui->k_gain->text();

    imp.m = M.toFloat();
    imp.B = B.toFloat();
    imp.K = K.toFloat();
    imp.cutoff_frequency = Cutoff.toFloat();
    imp.k_gain = k_gain.toFloat();

    ui->Mass->setText(QString::number(imp.m));
    ui->Damper->setText(QString::number(imp.B));
    ui->Spring->setText(QString::number(imp.K));
    ui->Cutoff_freq->setText(QString::number(imp.cutoff_frequency));
    ui->k_gain->setText(QString::number(imp.k_gain));
}


