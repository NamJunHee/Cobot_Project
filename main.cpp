#include "mainwindow.h"

#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;
    //while(1)
    //{w.serial();}
    w.show();
    return a.exec();
}
