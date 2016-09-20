#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QtWidgets>

MainWindow::MainWindow(QWidget *parent) :
    QWidget(parent)/*,
    ui(new Ui::MainWindow)*/
{
    //ui->setupUi(this);
    statuslabelLayout = new QHBoxLayout();
    statusLabel = new QLabel(tr("Listening for TDOA UDP server messages"));
    statusLabel->setWordWrap(true);
    statuslabelLayout->addWidget(statusLabel);

    statusLabel2 = new QLabel(tr(""));
    statusLabel2->setWordWrap(true);
    statuslabelLayout->addWidget(statusLabel2);


    AnchorlabelLayout = new QHBoxLayout();
    AnchorXLabel = new QLabel(tr("Anchor X:    "));
    AnchorYLabel = new QLabel(tr("Anchor Y:    "));
    AnchorZLabel = new QLabel(tr("Anchor Z:    "));

    AnchorlabelLayout->addWidget(AnchorXLabel);
    AnchorlabelLayout->addWidget(AnchorYLabel);
    AnchorlabelLayout->addWidget(AnchorZLabel);

    stopButton = new QPushButton(tr("&Stop"));
    startButton = new QPushButton(tr("&Start"));

    buttonLayout = new QHBoxLayout;
    //buttonLayout->addStretch(1);
    buttonLayout->addWidget(startButton);
    buttonLayout->addWidget(stopButton);
    //buttonLayout->addStretch(1);

    mainLayout = new QVBoxLayout;
    mainLayout->addLayout(statuslabelLayout);
    mainLayout->addLayout(AnchorlabelLayout);
    mainLayout->addLayout(buttonLayout);

    //mainLayout->setAlignment(statuslabelLayout,Qt::AlignLeft);
    //mainLayout->setAlignment(AnchorlabelLayout,Qt::AlignLeft);
    mainLayout->setAlignment(buttonLayout,Qt::AlignRight);

    setLayout(mainLayout);

    stopButton->setDisabled(true);

    //Технология пространственного трекинга для систем виртуальной реальности (VR)
    setWindowTitle(tr("VRST"));//VR space tracking

    resize(480,160);
    setFixedSize(this->size());

    //start network
    ntw = new Ntw(this);
    //create algorithm instance
    alg = new Alg(this);
    //alg = new Alg();

    connect(stopButton, SIGNAL(clicked()), this, SLOT(stop()));
    connect(startButton, SIGNAL(clicked()), this, SLOT(start()));
}

MainWindow::~MainWindow()
{
    delete statusLabel;
    delete statusLabel2;
    delete AnchorXLabel;
    delete AnchorYLabel;
    delete AnchorZLabel;
    delete stopButton;
    delete startButton;

    delete statuslabelLayout;
    delete AnchorlabelLayout;
    delete buttonLayout;
    delete mainLayout;

    //delete ui;
    delete ntw;
    delete alg;
}

void MainWindow::start()
{
    ntw->start();
    startButton->setDisabled(true);
    stopButton->setDisabled(false);
    //alg->DirectCalculationMethod();
}

void MainWindow::stop()
{
    ntw->stop();
    startButton->setDisabled(false);
    stopButton->setDisabled(true);
}

void MainWindow::SetOutput(QString txt,QString txt2,QString txt3,QString txt4,QString txt5)
{
    SetLabelText(txt);
    SetLabelText2(txt2);
    SetLabelTextAnchorXLabel(txt3);
    SetLabelTextAnchorYLabel(txt4);
    SetLabelTextAnchorZLabel(txt5);
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



