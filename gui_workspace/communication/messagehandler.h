#ifndef MESSAGEHANDLER_H
#define MESSAGEHANDLER_H

#include <functional>
#include <unordered_map>
#include <string>

#include "messagetypes.h"
#include "protocolconstants.h"
#include "protocolparser.h"

#include <QByteArray>

class MessageHandler {
public:
    MessageHandler();

    void setupHandlers();

    void handleMessage(
        std::vector<uint8_t> message,
        ProtocolConstants::RobotMessageType& messageType,
        ProtocolConstants::Responses& responseType,
        std::vector<uint8_t>& payload,
        DataVariant& output
    );

private:
    using HandlerFunction = std::function<void(std::vector<uint8_t>, DataVariant&)>;

    void registerMessageHandler(ProtocolConstants::RobotMessageType messageType, HandlerFunction handler);
    void handleConnect(std::vector<uint8_t> payload, DataVariant& output);
    void handleDisconnect(std::vector<uint8_t> payload, DataVariant& output);
    void handleHome(std::vector<uint8_t> payload, DataVariant& output);
    void handleDisable(std::vector<uint8_t> payload, DataVariant& output);
    void handleReadJointAngles(std::vector<uint8_t> payload, DataVariant& output);
    void handleUpdateEEPos(std::vector<uint8_t> payload, DataVariant& output);
    void handleSaveCurrentPosition(std::vector<uint8_t> payload, DataVariant& output);
    void handleMoveToPosition(std::vector<uint8_t> payload, DataVariant& output);
    void handlePlayBack(std::vector<uint8_t> payload, DataVariant& output);

    std::unordered_map<ProtocolConstants::RobotMessageType, HandlerFunction> message_handlers;

    ProtocolParser* parser;
};

#endif // MESSAGEHANDLER_H
