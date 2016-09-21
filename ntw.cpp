//#include <QtWidgets>
#include <QtNetwork>

#include "ntw.h"


Ntw::Ntw(MainWindow* wnd)
{
    mainWindow = (MainWindow*)wnd;
    udpSocket = new QUdpSocket(this);
    //udpSocket->bind(50010, QUdpSocket::ShareAddress);

    //connect(udpSocket, SIGNAL(readyRead()),
    //        this, SLOT(processPendingDatagrams()));
    datagram = new ANC_MSG();
}

Ntw::~Ntw()
{
    if(udpSocket->isOpen())
        udpSocket->abort();
    delete udpSocket;
    delete datagram;
}

void Ntw::processPendingDatagrams()
{
    qint64 sizeDatagramRead;
    int anchor_number;
    quint16 senderPort;
    int sync_series_number;
    QHostAddress sender;
    POINT3D retPoint;

    while (udpSocket->hasPendingDatagrams())
    {
        //QByteArray datagram;
        //datagram.resize(udpSocket->pendingDatagramSize());
        //udpSocket->readDatagram(datagram.data(), datagram.size(), &sender, &senderPort);

        sizeDatagramRead = udpSocket->readDatagram((char *)datagram, (qint64)sizeof(ANC_MSG), &sender, &senderPort);
        //qint64 readDatagram(char *data, qint64 maxlen, QHostAddress *host = 0, quint16 *port = 0);

        anchor_number = datagram->addr;
        sync_series_number = datagram->sync_n;

        //if datagram code is data packet signature and bytesRead > 0 then go to processing datagram
        if(datagram->code == ANC_REP_CODE && sizeDatagramRead>0)
            mainWindow->getAlg()->ProcessAnchorDatagram(datagram, &retPoint);

        //mainWindow->SetOutput(tr("Anchor: %1").arg(anchor_number),tr("Sync series_number: %1").arg(sync_series_number),tr("Anchor X: %1").arg(retPoint.x),
          //                    tr("Anchor Y: %1").arg(retPoint.y),tr("Anchor Z: %1").arg(retPoint.z));
        mainWindow->SetLabelText(tr("Anchor: %1").arg(anchor_number));
        mainWindow->SetLabelText2(tr("Sync series_number: %1").arg(sync_series_number));
    }
}

void Ntw::stop()
{
    udpSocket->abort();
    //mainWindow->SetLabelText(tr("Listening for TDOA UDP server messages"));
    mainWindow->SetLabelTextAnchorXLabel(tr("Anchor X:    "));
    mainWindow->SetLabelTextAnchorYLabel(tr("Anchor Y:    "));
    mainWindow->SetLabelTextAnchorZLabel(tr("Anchor Z:    "));
}

void Ntw::start()
{
    QString addrs = "192.168.1.56";
    QHostAddress addr;
    addr.setAddress("192.168.1.56");

    //udpSocket->bind(addr, 50010, QUdpSocket::ShareAddress);
    udpSocket->bind(QHostAddress::Any, 50010, QUdpSocket::ShareAddress);


    connect(udpSocket, SIGNAL(readyRead()),
            this, SLOT(processPendingDatagrams()));
}

