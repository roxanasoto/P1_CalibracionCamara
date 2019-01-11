#-------------------------------------------------
#
# Project created by QtCreator 2016-12-11T14:46:59
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = RCirculos
TEMPLATE = app


SOURCES += main.cpp\
        mainwindow.cpp \
    KFTracking.cpp \
    trackingGrid.cpp \
    patterndetector.cpp \
    cameracalibrator.cpp

HEADERS  += mainwindow.h \
    image.h \
    KFTracking.h \
    trackingGrid.h \
    constantParams.h \
    Geometria.h \
    Statistics.h \
    patterndetector.h \
    cameracalibrator.h

FORMS    += mainwindow.ui

#To check included Path :  	$ pkg-config --cflags opencv
#To check library :  		$ pkg-config --libs opencv

# considerar que el archivo cminpack.pc puede encontrarse en /usr/local/lib64/pkg-config,
# copiarlo a la ruta /usr/local/lib/pkg-config
# considerar que el archivo libcminpack.a puede encontrarse en /usr/local/lib64,
# copiarlo a la ruta /usr/local/lib

INCLUDEPATH += /usr/local/include/opencv
INCLUDEPATH += /usr/local/include/cminpack-1.3.6

LIBS += `pkg-config --cflags --libs opencv cminpack`

# -L represents for Directory
# -l represents for file
