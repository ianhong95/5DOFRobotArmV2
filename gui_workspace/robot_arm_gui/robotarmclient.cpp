#include "robotarmclient.h"
#include <QDebug>

RobotArmClient::RobotArmClient(QObject *parent)
    // Member initialization list
    : QObject(parent), socket(new QTcpSocket(this))
{
    // This block is what runs when the object is created. In this case, it just connects signals to slots.

    // When the socket emits the connected signal, the onconnected() method of this class will be called
    connect(socket, &QTcpSocket::connected, this, &RobotArmClient::onConnected);
}

void RobotArmClient::connectToServer(const QString host, quint16 port)
{
    socket->connectToHost(host, port);
}

void RobotArmClient::sendCommand(const QString command)
{
    if (socket->state() == QAbstractSocket::ConnectedState) {
        socket->write(command.toUtf8());
    }
}

void RobotArmClient::onConnected()
{
    socket->write("Holy shit it worked!");
}
