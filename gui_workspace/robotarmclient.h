#ifndef ROBOTARMCLIENT_H
#define ROBOTARMCLIENT_H

#include <QTcpSocket>
#include <QObject>

#include "protocolparser.h"
#include "messagetypes.h"
#include "messagehandler.h"

#include <optional>
#include <unordered_map>
#include <functional>
#include <string>


class RobotArmClient : public QObject {
    Q_OBJECT    // Macro that enables special Qt features (signals/slots)

public:
    RobotArmClient(QObject *parent = nullptr);

    ProtocolParser* parser;
    MessageHandler* messageHandler;

    bool connectionFlag = false;

    using SignalFunction = std::function<void(DataVariant data)>;

    bool connectToServer(const QString host, quint16 port);
    void disconnectFromServer();
    // void sendMessage(ProtocolConstants::RobotMessageType message, std::optional<uint8_t> payload = std::nullopt);
    void sendMessage(std::vector<uint8_t> fullMessage);

private slots:
    void onConnected();
    void onDisconnected();
    void onConnError(QAbstractSocket::SocketError socketError);
    void onBytesRecvd();

private:
    QTcpSocket* socket;
    ProtocolParser* protocol;

    std::unordered_map<ProtocolConstants::RobotMessageType, SignalFunction> signalsMap;

    // Setup functions
    void setupSignals();
    void registerSignal(ProtocolConstants::RobotMessageType messageType, SignalFunction signalFunction);

    // Emit handlers
    void emitSignal(ProtocolConstants::RobotMessageType messageType, DataVariant signalData);
    void emitJointAnglesRecvd(DataVariant signalData);
    void emitXYZPositionRecvd(DataVariant signalData);
    void emitHome(DataVariant signalData);
    void emitDisable(DataVariant signalData);
    void emitDisconnect(DataVariant signalData);

signals:
    void connError(const QString &errorMsg);
    void connStatusChanged(bool connected);
    void jointAnglesRecvd(JointAngles jointAngles);
    void xyzPositionRecvd(XYZPosition xyzPosition);
};

#endif // ROBOTARMCLIENT_H
