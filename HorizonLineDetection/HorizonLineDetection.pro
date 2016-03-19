#-------------------------------------------------
#
# Project created by QtCreator 2016-03-15T08:47:52
#
#-------------------------------------------------

QT       += core

QT       -= gui

TARGET = HorizonLineDetection
CONFIG   += console
CONFIG   -= app_bundle
CONFIG   += c++11

CPPFLAGS = -L/path/to/opencv3.0.0/lib \
           -I/path/to/opencv3.0.0/include

INCLUDEPATH += -I/home/paghdv/opencv/opencv-3.1.0/bin/include

TEMPLATE = app


LIBS += -L/home/paghdv/opencv/opencv-3.1.0/bin/lib \
        -lopencv_core \
        -lopencv_imgproc \
        -lopencv_highgui \
        -lopencv_features2d \
        -lopencv_calib3d \
        -lopencv_nonfree \
        -lopencv_video \
        -lopencv_ml \

SOURCES += main.cpp \
    ../src/HorizonLineDetector.cpp

HEADERS += \
    ../src/HorizonLineDetector.h
