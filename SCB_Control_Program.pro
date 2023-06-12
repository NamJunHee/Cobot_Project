QT       += core gui serialport opengl

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets printsupport
CONFIG += c++11

# You can make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
    delta_forward_kinematics.cpp \
    impedance_control_delta.cpp \
    main.cpp \
    mainwindow.cpp \
    mobile_kinematics.cpp \
    qcustomplot.cpp

HEADERS += \
    delta_forward_kinematics.h \
    impedance_control_delta.h \
    mainwindow.h \
    mobile_kinematics.h \
    qcustomplot.h

INCLUDEPATH += /usr/include/eigen3

LIBS += -lGL -lGLU

FORMS += \
    mainwindow.ui

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target
