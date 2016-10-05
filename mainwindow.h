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
//#include "alg.h"
//#include "alg1.h"
//#include "Alg2.h"

QT_BEGIN_NAMESPACE
class QLabel;
class QPushButton;
class QAction;
QT_END_NAMESPACE

class Ntw;
//class Alg;
class Alg1;
class Alg2;

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
	void SetOutput2(QString txt, QString txt2, QString txt3);
    void SetLabelText(QString txt);
    void SetLabelText2(QString txt);
    void SetLabelTextAnchorXLabel(QString txt);
    void SetLabelTextAnchorYLabel(QString txt);
    void SetLabelTextAnchorZLabel(QString txt);
	Alg1* getAlg1();
	Alg2* getAlg2();
	inline FILE* getCoordFile1() { return file_out; }
	inline FILE* getCoordFile2() { return file_out2; }

private:
    Ui::MainWindow *ui;
    QWidget *widget;
    QVBoxLayout *mainLayout;
    QHBoxLayout *buttonLayout;
    QHBoxLayout *statuslabelLayout;
	QHBoxLayout *labelLayout2;
    QHBoxLayout *AnchorlabelLayout;

    QLabel *statusLabel;
    QLabel *statusLabel2;
	QLabel *alg1Label;
	QLabel *alg2Label;
    QLabel *AnchorXLabel;
    QLabel *AnchorYLabel;
    QLabel *AnchorZLabel;
	QLabel *AnchorX2Label;
	QLabel *AnchorY2Label;
	QLabel *AnchorZ2Label;
    QPushButton *startButton;
    QPushButton *stopButton;

    Ntw* ntw;//network class
    Alg1* alg1;//algorithm class
	Alg2* alg2;

    bool state;
	FILE *file_out;
	FILE *file_out2;

private slots:
    void stop();
    void start();
};

#endif // MAINWINDOW_H
