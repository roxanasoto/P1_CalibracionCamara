#-------------------------------------------------
#
# Project created by QtCreator 2016-12-15T20:00:55
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = Calibration
TEMPLATE = app
INCLUDEPATH += /usr/local/include/opencv
INCLUDEPATH += /usr/local/include
#LIBS += -L/usr/local/lib -lopencv_core -lopencv_imgcodecs -lopencv_highgui -lopencv_imgproc -lopencv_videoio
#CONFIG += link_pkgconfig
#PKGCONFIG += opencv
LIBS += `pkg-config --cflags --libs opencv`
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

CONFIG += c++11

SOURCES += \
        main.cpp \
        mainwindow.cpp

HEADERS += \
        mainwindow.h

FORMS += \
        mainwindow.ui

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target

#SOURCES += main.cpp\
        #mainwindow.cpp

#HEADERS  += mainwindow.h

#FORMS    += mainwindow.ui

#INCLUDEPATH += /usr/local/include/opencv
#INCLUDEPATH += /usr/local/include
#LIBS += `pkg-config --cflags --libs opencv`
#LIBS += -L/usr/local/lib -lopencv_core -lopencv_imgcodecs -lopencv_highgui
