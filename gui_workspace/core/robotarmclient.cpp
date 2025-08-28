/* INCOMING MESSAGE MANAGER
 *
 * */

#include "robotarmclient.h"
#include "protocolparser.h"

#include <QDebug>
#include <QJsonObject>
#include <QJsonArray>
#include <QJsonDocument>
#include <QVariant>
#include <QList>
#include <QByteArray>
#include <QBuffer>

#include <iostream>
#include <optional>
#include <functional>


RobotArmClient::RobotArmClient(QObject *parent)
    // Member initialization list
    : QObject(parent), socket(new QTcpSocket(this))
{
    // This block is what runs when the object is created. In this case, it just connects signals to slots.
    parser = new ProtocolParser();
    messageHandler = new MessageHandler();
    setupSignals();

    // When the socket emits the connected signal, the onConnected() method of this class will be called
    connect(socket, &QTcpSocket::connected, this, &RobotArmClient::onConnected);
    connect(socket, &QTcpSocket::disconnected, this, &RobotArmClient::onDisconnected);
    connect(socket, &QTcpSocket::errorOccurred, this, &RobotArmClient::onConnError);
    connect(socket, &QTcpSocket::readyRead, this, &RobotArmClient::onBytesRecvd);
}

void RobotArmClient::setupSignals() {
    // RobotArmClient::registerSignal(ProtocolConstants::RobotMessageType::Disconnect, RobotArmClient::onDisconnected());
    RobotArmClient::registerSignal(ProtocolConstants::RobotMessageType::ReadJointAngles, [this](DataVariant signalData) {
        this->emitJointAnglesRecvd(signalData);
    });
    RobotArmClient::registerSignal(ProtocolConstants::RobotMessageType::UpdateEEPos, [this](DataVariant signalData) {
        this->emitXYZPositionRecvd(signalData);
    });
    RobotArmClient::registerSignal(ProtocolConstants::RobotMessageType::Home, [this](DataVariant signalData) {
        this->emitHome(signalData);
    });
    RobotArmClient::registerSignal(ProtocolConstants::RobotMessageType::Disable, [this](DataVariant signalData) {
        this->emitDisable(signalData);
    });
    RobotArmClient::registerSignal(ProtocolConstants::RobotMessageType::SaveCurrentPosition, [this](DataVariant signalData) {
        this->emitSavePosRespRecvd(signalData);
    });
}

void RobotArmClient::registerSignal(ProtocolConstants::RobotMessageType messageType, RobotArmClient::SignalFunction signalFunction) {
    RobotArmClient::signalsMap[messageType] = signalFunction;
}

void RobotArmClient::emitSignal(ProtocolConstants::RobotMessageType messageType, DataVariant signalData) {
    auto it = RobotArmClient::signalsMap.find(messageType);
    it->second(signalData);
}

bool RobotArmClient::connectToServer(const QString host, quint16 port) {
    try {
        socket->connectToHost(host, port);
        if (RobotArmClient::connectionFlag == true) {
            return true;
        }
        else {
            return false;
        }
    }
    catch (int errorCode) {
        std::cout<< "Error occured: " << errorCode;
        return false;
    }
}

void RobotArmClient::disconnectFromServer() {
    std::vector<uint8_t> disableMessage = parser->encodeMessage(ProtocolConstants::RobotMessageType::Disable);
    std::vector<uint8_t> disconnectMessage = parser->encodeMessage(ProtocolConstants::RobotMessageType::Disconnect);
    RobotArmClient::sendMessage(disableMessage);
    RobotArmClient::sendMessage(disconnectMessage);
    socket->disconnectFromHost();
}

void RobotArmClient::onConnected() {
    std::vector<uint8_t> homeMessage = parser->encodeMessage(ProtocolConstants::RobotMessageType::Home);
    // RobotArmClient::sendMessage(homeMessage);
    RobotArmClient::connectionFlag = true;
    emit connStatusChanged(true);
}

void RobotArmClient::onDisconnected() {
    RobotArmClient::connectionFlag = false;
    emit connStatusChanged(false);
}


/* =============
 * COMMUNICATION
 * =============
 */

void RobotArmClient::sendMessage(std::vector<uint8_t> fullMessage) {
    // std::vector<uint8_t> messageByteArray = parser->encodeMessage(messageType, payload);
    QByteArray qtMessage(reinterpret_cast<const char*>(fullMessage.data()), static_cast<int>(fullMessage.size())); // Convert to QByteArray
    if (sizeof(qtMessage) > 0) {
        socket->write(qtMessage);
    }
}

void RobotArmClient::onBytesRecvd() {
    QByteArray incomingByteArray = socket->read(ProtocolConstants::DATA_FRAME_LENGTH);
    std::vector<uint8_t> incomingData(incomingByteArray.begin(), incomingByteArray.end());  // Copy the incoming data to a std::vector

    if (sizeof(incomingByteArray >= ProtocolConstants::DATA_FRAME_LENGTH)) {
        ProtocolConstants::RobotMessageType messageType;
        ProtocolConstants::Responses responseType;
        std::vector<uint8_t> payload;
        DataVariant output;

        messageHandler->handleMessage(incomingData, messageType, responseType, payload, output);

        emitSignal(messageType, output);
    }
}

/* =============
 * EMIT HANDLERS
 * =============
 */

void RobotArmClient::emitJointAnglesRecvd(DataVariant signalData) {
    JointAngles jointAngles = std::get<JointAngles>(signalData);
    emit jointAnglesRecvd(jointAngles);
}

void RobotArmClient::emitXYZPositionRecvd(DataVariant signalData) {
    XYZPosition xyzPosition = std::get<XYZPosition>(signalData);
    emit xyzPositionRecvd(xyzPosition);
}

void RobotArmClient::emitHome(DataVariant signalData) {
    JointAnglesAndPosition anglesAndPosition = std::get<JointAnglesAndPosition>(signalData);
    emit jointAnglesRecvd(anglesAndPosition.angles);
    emit xyzPositionRecvd(anglesAndPosition.coordinates);
}

void RobotArmClient::emitDisable(DataVariant signalData) {}

void RobotArmClient::emitDisconnect(DataVariant signalData) {}

void RobotArmClient::emitSavePosRespRecvd(DataVariant signalData) {
    SavedXYZPosition savedXYZPositionData = std::get<SavedXYZPosition>(signalData);
    emit savePosRespRecvd(savedXYZPositionData);
}

/* ===============
 * ERROR HANDLING
 * ===============
 */

void RobotArmClient::onConnError(QAbstractSocket::SocketError socketError) {
    QString errorMsg = socket->errorString();
    emit connError(errorMsg);
}
