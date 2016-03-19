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

INCLUDEPATH += /home/paghdv/opencv/opencv-3.1.0/install/include/

TEMPLATE = app

LIBS += -L/home/paghdv/opencv/opencv-3.1.0/install/lib -lopencv_core \
        -L/home/paghdv/opencv/opencv-3.1.0/install/lib -lopencv_imgproc \
        -L/home/paghdv/opencv/opencv-3.1.0/install/lib -lopencv_highgui \
        -L/home/paghdv/opencv/opencv-3.1.0/install/lib -lopencv_features2d \
        -L/home/paghdv/opencv/opencv-3.1.0/install/lib -lopencv_calib3d \
        -L/home/paghdv/opencv/opencv-3.1.0/install/lib -lopencv_xfeatures2d \
        -L/home/paghdv/opencv/opencv-3.1.0/install/lib -lopencv_video \
        -L/home/paghdv/opencv/opencv-3.1.0/install/lib -lopencv_videoio \
        -L/home/paghdv/opencv/opencv-3.1.0/install/lib -lopencv_ml \
        -L/home/paghdv/opencv/opencv-3.1.0/install/lib -lopencv_imgcodecs \

SOURCES += main.cpp \
    ../src/HorizonLineDetector.cpp

HEADERS += \
    ../src/HorizonLineDetector.h
