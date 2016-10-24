#include "mainwindow.h"
#include "alg1.h"
#include "Alg2.h"
#include "ui_mainwindow.h"

#include <QtWidgets>

MainWindow::MainWindow(QWidget *parent) :
	QWidget(parent), /*QMainWindow(parent),*/
    ui(new Ui::MainWindow)
{
    //ui->setupUi((QMainWindow*)this);
    statuslabelLayout = new QHBoxLayout();
    statusLabel = new QLabel(tr("Listening for TDOA UDP server messages"));
    statusLabel->setWordWrap(true);
    statuslabelLayout->addWidget(statusLabel);

    statusLabel2 = new QLabel(tr(""));
    statusLabel2->setWordWrap(true);
    statuslabelLayout->addWidget(statusLabel2);

	QFrame* line = new QFrame();
	line->setFrameShape(QFrame::HLine);
	line->setFrameShadow(QFrame::Sunken);	
	QFrame* line2 = new QFrame();
	line2->setFrameShape(QFrame::HLine);
	line2->setFrameShadow(QFrame::Sunken);

	labelLayout2 = new QHBoxLayout();
	alg2Label = new QLabel(tr("Algoritm 2"));
	alg2Label->setWordWrap(true);	
	labelLayout2->addWidget(alg2Label);

	QLabel* algLabel = new QLabel(tr("Algoritm 1"));
	algLabel->setWordWrap(true);

    AnchorlabelLayout = new QHBoxLayout();
    AnchorXLabel = new QLabel(tr("Anchor X:    "));
    AnchorYLabel = new QLabel(tr("Anchor Y:    "));
    AnchorZLabel = new QLabel(tr("Anchor Z:    "));
    AnchorlabelLayout->addWidget(AnchorXLabel);
    AnchorlabelLayout->addWidget(AnchorYLabel);
    AnchorlabelLayout->addWidget(AnchorZLabel);

	QHBoxLayout* AnchorlabelLayout2 = new QHBoxLayout();
	AnchorX2Label = new QLabel(tr("Anchor X:    "));
	AnchorY2Label = new QLabel(tr("Anchor Y:    "));
	AnchorZ2Label = new QLabel(tr("Anchor Z:    "));
	AnchorlabelLayout2->addWidget(AnchorX2Label);
	AnchorlabelLayout2->addWidget(AnchorY2Label);
	AnchorlabelLayout2->addWidget(AnchorZ2Label);

    stopButton = new QPushButton(tr("&Stop"));
    startButton = new QPushButton(tr("&Start"));

    buttonLayout = new QHBoxLayout();
    buttonLayout->addWidget(startButton);
    buttonLayout->addWidget(stopButton);    

    mainLayout = new QVBoxLayout(this);
    mainLayout->addLayout(statuslabelLayout);
	mainLayout->addWidget(line);
	mainLayout->addWidget(algLabel);
    mainLayout->addLayout(AnchorlabelLayout);
	mainLayout->addWidget(line2);
	mainLayout->addLayout(labelLayout2);
	mainLayout->addLayout(AnchorlabelLayout2);
	//mainLayout->addSpacing(40);
    mainLayout->addLayout(buttonLayout);
	

    mainLayout->setAlignment(buttonLayout,Qt::AlignRight);

    setLayout(mainLayout);

    stopButton->setDisabled(true);

    //Технология пространственного трекинга для систем виртуальной реальности (VR)
    setWindowTitle(tr("VR space tracking"));//VR space tracking

    resize(480,200);
    setFixedSize(this->size());

    //start network
    ntw = new Ntw(this);
    //create algorithm instances
    alg1 = new Alg1(this);
    alg2 = new Alg2(this);
	file_out = fopen("coordinates_out.txt", "wt");
	file_out2 = fopen("coordinates_out2.txt", "wt");
	file_raw_data = fopen("raw_data.txt", "wt");	

    connect(stopButton, SIGNAL(clicked()), this, SLOT(stop()));
    connect(startButton, SIGNAL(clicked()), this, SLOT(start()));
}

MainWindow::~MainWindow()
{
	delete ui;
    /*delete statusLabel;
    delete statusLabel2;
    delete AnchorXLabel;
    delete AnchorYLabel;
    delete AnchorZLabel;
    delete stopButton;
    delete startButton;

    delete statuslabelLayout;
    delete AnchorlabelLayout;
    delete buttonLayout;
    delete mainLayout;*/
    
    //delete ntw;
    //delete alg;
/*	if(file_out)
		fclose(file_out);
	if (file_out2)
		fclose(file_out2);*/
}

Alg1* MainWindow::getAlg1()
{
	return alg1;
}

Alg2* MainWindow::getAlg2()
{
	return alg2;
}

void MainWindow::start()
{
    ntw->start();
    startButton->setDisabled(true);
    stopButton->setDisabled(false);
    //alg->DirectCalculationMethod();
	file_out = fopen("coordinates_out.txt", "wt");
	file_out2 = fopen("coordinates_out2.txt", "wt");
	file_raw_data = fopen("raw_data.txt", "wt");
	
	qDebug("start");
}

void MainWindow::stop()
{
    ntw->stop();
    startButton->setDisabled(false);
    stopButton->setDisabled(true);
	if (file_out)
		fclose(file_out);
	if (file_out2)
		fclose(file_out2);
	if (file_raw_data)
		fclose(file_raw_data);
	qDebug("stop");
}

void MainWindow::SetOutput(QString txt,QString txt2,QString txt3,QString txt4,QString txt5)
{
    SetLabelText(txt);
    SetLabelText2(txt2);
    SetLabelTextAnchorXLabel(txt3);
    SetLabelTextAnchorYLabel(txt4);
    SetLabelTextAnchorZLabel(txt5);
}

void MainWindow::SetOutput2(QString txt, QString txt2, QString txt3)
{
	AnchorX2Label->setText(txt);
	AnchorY2Label->setText(txt2);
	AnchorZ2Label->setText(txt3);	
}

void MainWindow::SetLabelText(QString txt)
{
    statusLabel->setText(txt);
}

void MainWindow::SetLabelText2(QString txt)
{
    statusLabel2->setText(txt);
}

void MainWindow::SetLabelTextAnchorXLabel(QString txt)
{
    AnchorXLabel->setText(txt);
}
void MainWindow::SetLabelTextAnchorYLabel(QString txt)
{
    AnchorYLabel->setText(txt);
}
void MainWindow::SetLabelTextAnchorZLabel(QString txt)
{
    AnchorZLabel->setText(txt);
}



