#-------------------------------------------------
#
# Project created by QtCreator 2016-09-02T13:04:56
#
#-------------------------------------------------

QT       += core gui opengl
QT += network widgets

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = VRST
TEMPLATE = app


SOURCES += main.cpp\
        mainwindow.cpp \
    ntw.cpp \
    alg.cpp \
    alg1.cpp \
    Alg2.cpp \
    oglwidget.cpp

HEADERS  += mainwindow.h \
    ntw.h \
    alg.h \
    alg1.h \
    Alg2.h \
    datagram.h \
    oglwidget.h

FORMS    += mainwindow.ui

LIBS += -lglu32 -lopengl32
