#ifndef NTW_H
#define NTW_H

#include <QWidget>
#include <QMainWindow>

#include "mainwindow.h"

#include "datagram.h"

class MainWindow;

QT_BEGIN_NAMESPACE
class QUdpSocket;
class QAction;
QT_END_NAMESPACE


class Ntw : public QWidget
{
    Q_OBJECT
public:
    Ntw(MainWindow* wnd = 0);
    virtual ~Ntw();
    void stop();
    void start();
private:

    QUdpSocket* udpSocket;
    MainWindow* mainWindow;
    ANC_MSG*  datagram;// received UDP packet
private slots:
    void processPendingDatagrams();
};

#endif // NTW_H
