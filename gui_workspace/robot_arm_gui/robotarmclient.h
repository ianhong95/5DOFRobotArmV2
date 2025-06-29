#ifndef ROBOTARMCLIENT_H
#define ROBOTARMCLIENT_H
#include <QTcpSocket>
#include <QObject>


class RobotArmClient : public QObject
{
    Q_OBJECT    // Macro that enables special Qt features (signals/slots)

public:
    RobotArmClient(QObject *parent = nullptr);
    void connectToServer(const QString host, quint16 port);
    void sendCommand(const QString command);

private slots:
    void onConnected();

private:
    QTcpSocket *socket;
};

#endif // ROBOTARMCLIENT_H
