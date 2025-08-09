#include "messagehandler.h"
#include "protocolparser.h"
#include "protocolconstants.h"
#include "messagetypes.h"
#include "robotarmclient.h"

#include <iostream>
#include <functional>
#include <unordered_map>
#include <stdint.h>
#include <vector>
#include <string>
#include <cstring>
#include <cstdint>

#include <QByteArray>

MessageHandler::MessageHandler() {
    parser = new ProtocolParser();
    setupHandlers();
}

void MessageHandler::setupHandlers() {
    MessageHandler::registerMessageHandler(ProtocolConstants::RobotMessageType::Connect, [this](std::vector<uint8_t> payload, DataVariant& output) {
        this->handleConnect(payload, output);
    });
    MessageHandler::registerMessageHandler(ProtocolConstants::RobotMessageType::Disconnect, [this](std::vector<uint8_t> payload, DataVariant& output) {
        this->handleDisconnect(payload, output);
    });
    MessageHandler::registerMessageHandler(ProtocolConstants::RobotMessageType::Home, [this](std::vector<uint8_t> payload, DataVariant& output) {
        this->handleHome(payload, output);
    });
    MessageHandler::registerMessageHandler(ProtocolConstants::RobotMessageType::Disable, [this](std::vector<uint8_t> payload, DataVariant& output) {
        this->handleDisable(payload, output);
    });
    MessageHandler::registerMessageHandler(ProtocolConstants::RobotMessageType::ReadJointAngles, [this](std::vector<uint8_t> payload, DataVariant& output) {
        this->handleReadJointAngles(payload, output);
    });
    MessageHandler::registerMessageHandler(ProtocolConstants::RobotMessageType::UpdateEEPos, [this](std::vector<uint8_t> payload, DataVariant& output) {
        this->handleUpdateEEPos(payload, output);
    });
    MessageHandler::registerMessageHandler(ProtocolConstants::RobotMessageType::SaveCurrentPosition, [this](std::vector<uint8_t> payload, DataVariant& output) {
        this->handleSaveCurrentPosition(payload, output);
    });
    MessageHandler::registerMessageHandler(ProtocolConstants::RobotMessageType::MoveToPosition, [this](std::vector<uint8_t> payload, DataVariant& output) {
        this->handleMoveToPosition(payload, output);
    });
    MessageHandler::registerMessageHandler(ProtocolConstants::RobotMessageType::PlayCurrentSequence, [this](std::vector<uint8_t> payload, DataVariant& output) {
        this->handlePlayCurrentSequence(payload, output);
    });
}

void MessageHandler::registerMessageHandler(ProtocolConstants::RobotMessageType messageType, MessageHandler::HandlerFunction handler) {
    MessageHandler::message_handlers[messageType] = handler;
}

// Handle incoming messages?
void MessageHandler::handleMessage(std::vector<uint8_t> message,
                                   ProtocolConstants::RobotMessageType& messageType,
                                   ProtocolConstants::Responses& responseType,
                                   std::vector<uint8_t>& payload,
                                   DataVariant& output
                                   ) {

    parser->decodeMessage(message, messageType, responseType, payload);   // messageType and payload are outputs

    auto it = MessageHandler::message_handlers.find(messageType);    // find the element in the map with key messageType
    it->second(payload, output);
}

void MessageHandler::handleConnect(std::vector<uint8_t> payload, DataVariant& output) {
    ProtocolConstants::Responses response;
}

void MessageHandler::handleDisconnect(std::vector<uint8_t> payload, DataVariant& output) {
    ProtocolConstants::Responses response;
}

void MessageHandler::handleHome(std::vector<uint8_t> payload, DataVariant& output) {
    JointAngles jointAngles;
    XYZPosition xyzPosition;

    std::memcpy(jointAngles.angles.data(), payload.data(), jointAngles.angles.size() * sizeof(float));
    std::memcpy(xyzPosition.coordinates.data(), payload.data() + 5 * sizeof(float), xyzPosition.coordinates.size() * sizeof(float));

    output = JointAnglesAndPosition{jointAngles, xyzPosition};
}

void MessageHandler::handleDisable(std::vector<uint8_t> payload, DataVariant &output) {

}

void MessageHandler::handleReadJointAngles(std::vector<uint8_t> payload, DataVariant& output) {
    JointAngles jointAngles;

    /*
     * .data() returns a pointer to the first element in the array.
     * We'll need to skip the first two elements (message type and response type).
     * The first two arguments are pointers. jointAngles.angles decays to a pointer to the first element because of some magic.
     * memcpy will start writing at the address of that first element then continue writing to the rest of the array.
    */
    std::memcpy(jointAngles.angles.data(), payload.data(), jointAngles.angles.size() * sizeof(float));

    output = jointAngles;   // std::vector<float>
}

void MessageHandler::handleUpdateEEPos(std::vector<uint8_t> payload, DataVariant &output) {
    XYZPosition xyzPosition;

    std::memcpy(xyzPosition.coordinates.data(), payload.data(), xyzPosition.coordinates.size() * sizeof(float));

    output = xyzPosition;
}

void MessageHandler::handleSaveCurrentPosition(std::vector<uint8_t> payload, DataVariant &output) {
    XYZPosition xyzPosition;

    int index = 0;

    std::memcpy(&index, payload.data(), sizeof(int));
    std::memcpy(xyzPosition.coordinates.data(), payload.data() + sizeof(int), xyzPosition.coordinates.size() * sizeof(float));

    output = SavedXYZPosition{index, "", xyzPosition};
}

void MessageHandler::handleMoveToPosition(std::vector<uint8_t> payload, DataVariant &output) {

}

void MessageHandler::handlePlayCurrentSequence(std::vector<uint8_t> payload, DataVariant &output) {

}
