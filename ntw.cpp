//#include <QtWidgets>
#include <QtNetwork>

#include "ntw.h"
#include "alg1.h"
#include "alg2.h"


Ntw::Ntw(MainWindow* wnd)
{
    mainWindow = (MainWindow*)wnd;
    udpSocket = new QUdpSocket(this);
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
	POINT3D retPoint2;

	retPoint.x = 0; retPoint.y = 0; retPoint.z = 0;
	retPoint2.x = 0; retPoint2.y = 0; retPoint2.z = 0;

    while (udpSocket->hasPendingDatagrams())
    {
        sizeDatagramRead = udpSocket->readDatagram((char *)datagram, (qint64)sizeof(ANC_MSG), &sender, &senderPort);       

        anchor_number = datagram->addr;
        sync_series_number = datagram->sync_n;

        //if datagram code is data packet signature and bytesRead > 0 then go to processing datagram
		if (datagram->code == ANC_REP_CODE && sizeDatagramRead > 0)
		{
			mainWindow->getAlg1()->ProcessAnchorDatagram(datagram, &retPoint);
			mainWindow->getAlg2()->ProcessAnchorDatagram(datagram, &retPoint2);			
		}
    }
}

void Ntw::stop()
{
    udpSocket->abort();
}

void Ntw::start()
{
    /*QHostAddress addr;
    addr.setAddress("192.168.1.56");
    udpSocket->bind(addr, 50010, QUdpSocket::ShareAddress);*/
    
	udpSocket->bind(QHostAddress::Any, 50010, QUdpSocket::ShareAddress);
    connect(udpSocket, SIGNAL(readyRead()),
            this, SLOT(processPendingDatagrams()));
}

