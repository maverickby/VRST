/****************************************************************************
** VRST, Virtual reality space tracking
** Copyright (C) Digital Gravitation 2016
** Contact:

**
****************************************************************************/

#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <stdlib.h>
#include <QMainWindow>
#include <QVBoxLayout>
#include "ntw.h"
#include "alg.h"

QT_BEGIN_NAMESPACE
class QLabel;
class QPushButton;
class QAction;
QT_END_NAMESPACE

class Ntw;
class Alg;

namespace Ui {
class MainWindow;
}

class MainWindow : public QWidget
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    inline QLabel *  getStatusLabel(){return statusLabel;}
    inline QLabel *  getStatusLabel2(){return statusLabel2;}
    void SetOutput(QString txt,QString txt2,QString txt3,QString txt4,QString txt5);
    void SetLabelText(QString txt);
    void SetLabelText2(QString txt);
    void SetLabelTextAnchorXLabel(QString txt);
    void SetLabelTextAnchorYLabel(QString txt);
    void SetLabelTextAnchorZLabel(QString txt);
    inline Alg* getAlg(){return alg;}
	inline FILE* getCoordFile() { return file_out; }

private:
    Ui::MainWindow *ui;
    QWidget *widget;
    QVBoxLayout *mainLayout;
    QHBoxLayout *buttonLayout;
    QHBoxLayout *statuslabelLayout;
    QHBoxLayout *AnchorlabelLayout;

    QLabel *statusLabel;
    QLabel *statusLabel2;
    QLabel *AnchorXLabel;
    QLabel *AnchorYLabel;
    QLabel *AnchorZLabel;
    QPushButton *startButton;
    QPushButton *stopButton;

    Ntw* ntw;//network class
    Alg* alg;//algorithm class

    bool state;
	FILE *file_out;

private slots:
    void stop();
    void start();
};

#endif // MAINWINDOW_H
