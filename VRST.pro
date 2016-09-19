#-------------------------------------------------
#
# Project created by QtCreator 2016-09-02T13:04:56
#
#-------------------------------------------------

QT       += core gui
QT += network widgets

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = VRST
TEMPLATE = app


SOURCES += main.cpp\
        mainwindow.cpp \
    ntw.cpp \
    alg.cpp

HEADERS  += mainwindow.h \
    ntw.h \
    alg.h

FORMS    += mainwindow.ui
