#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#define Transport_mode 1
#define Precison_mode  3

#include <QMainWindow>
#include <QSerialPort>
#include <QSerialPortInfo>
#include <QString>
#include <stdlib.h>
#include <math.h>
#include <QTimer>
#include <cstdio>
#include <QList>
#include <QComboBox>
#include <iostream>
#include <qcustomplot.h>
#include <impedance_control_delta.h>
#include <delta_forward_kinematics.h>
#include <mobile_kinematics.h>
#include <QtMath>
#include <QFile>
#include <QTextStream>
#include <QKeyEvent>
#include <complex.h>

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
    Impedance_Control_Delta imp = Impedance_Control_Delta();
    delta_forward_kinematics forward = delta_forward_kinematics();
    Mobile_Kinematics mk = Mobile_Kinematics();
    //Mobile_kinematics mk = Mobile_Kinematics();
    void serial();

public slots:
    void serialReceived();
    void serialSend();
    void calRun();
    void update();
    void IMUCallback();

private slots:
    void on_serial_port_set_clicked();

    void on_serial_connect_clicked();

    void on_serial_disconnect_clicked();

    void on_impedance_clicked();

    void on_position_clicked();

    void on_write_tau_toggled(bool checked);

    void on_write_pos_toggled(bool checked);

    void on_Trajectory_clicked();

    void on_UI_Button_toggled(bool checked);

    void on_pushButton_3_clicked();

    void Positon_Init();

private:
    Ui::MainWindow *ui;
    QTimer *_100ms_Timer,*_50ms_Timer,*_20ms_Timer, *_10ms_Timer, *_5ms_Timer,*_2ms_Timer,*_1ms_Timer;
    QTimer *delay_time;
    QSerialPort *Dserial, *Mserial, *Matserial, *IMUSerial;
    QCustomPlot *customPlot;
    //Impedance_Control *imp;
    int Connect_Sign = 0;
    int time = 10;
    double Encoder1, Encoder2, Encoder3, EncoderY, EncoderR, EncoderP ,Encoder_d3 = 0;
    double TorqueOut1,TorqueOut2,TorqueOut3 = 0;
    double LoadCell1, LoadCell2, LoadCell3 = 0;
    double LoadCell1_temp, LoadCell2_temp, LoadCell3_temp = 0;
    double load1_weight = 0;
    double load2_weight = 0;
    double load3_weight = 0;

    double time_cnt = 0;
    int G_compensation_flag= 0;

    double M = 0;
    Vector3d TorqueOut;
    int i = 0;
    int cnt = 0;
    int mode_flag = 0;
    int SW_Mode = 0;
    int write_tau_flag = 0;
    int write_pos_flag = 0;
    int ui_flag = 0;

    int sort_count = 64;

    QString Send_Angle(double q1, double q2, double q3, int F_flag1, int F_flag2, int F_flag3);
    QString Send_RPM(float w1, float w2, float w3, float w4);

    void Plot_init();
    void write(const QString filename);
    void read (const QString filename);
    double Loadcell_Linealization(double x);
    double Loadcell1_Calibration(double x);
    double Loadcell2_Calibration(double x);
    double Loadcell3_Calibration(double x);

    QString tau_Txt = "/home/capstone/data/torque_data.txt";
    QString pos_Txt = "/home/capstone/data/position_data.txt";

    QFile tau_file;//("/home/capstone/data/torque_data.txt");
    QFile pos_file;//("/home/capstone/data/position_data.txt");

    QTextStream tau_out;
    QTextStream pos_out;

    void keyPressEvent(QKeyEvent *event);

    double End_Point;

    QByteArray Rx_data;
    QByteArray Rx_sort_data;
    QByteArray Tx_data;
    unsigned char TxBuffer[10] = {0,};
    int a_flag = 0;
    int a_array = 0;
    int load_var = 28;
    float m_limit = 0.4;
    const unsigned char A  = 0x61;
    const unsigned char N  = 0x6E;
    const unsigned char G  = 0x67;
    const unsigned char CR = 0x0D;
    const unsigned char LF = 0x0A;
    const unsigned char Y  = 0x79;
    const unsigned char R  = 0x72;
    const unsigned char SPACE = 0x32;
    int init_flag = 1;

    typedef struct{
        float roll;
        float pitch;
        float yaw;
        float roll_vel;
        float pitch_vel;
        float yaw_vel;
    }IMU ;

    IMU imu;
    void IMU_init();

};
#endif // MAINWINDOW_H
