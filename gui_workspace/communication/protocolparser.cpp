#include "protocolparser.h"
#include "protocolconstants.h"
#include "messagetypes.h"

#include <iostream>
#include <vector>
#include <cstdint>
#include <string>
#include <cstring>
#include <algorithm>

ProtocolParser::ProtocolParser() {

}

// Usage: Command is received from GUI and we need to decide what to do with it.
// GUI command -> messagehandler -> encode message -> messagehandler -> socket server
// TODO: Make this function pure C++ and move Qt elements to the client

// Encode an optional uint8_t
std::vector<uint8_t> ProtocolParser::encodeMessage(ProtocolConstants::RobotMessageType messageType) {
    uint8_t encodedMessageType = static_cast<uint8_t>(messageType);
    std::vector<uint8_t> messageByteArray;

    messageByteArray.insert(messageByteArray.begin(), encodedMessageType);

    messageByteArray.resize(ProtocolConstants::DATA_FRAME_LENGTH, ProtocolConstants::PADDING_CHAR);     // Pad with null bytes

    return messageByteArray;
}

// Encode a vector of uint8_t
std::vector<uint8_t> ProtocolParser::encodeMessage(ProtocolConstants::RobotMessageType messageType, std::vector<uint8_t> payload) {
    uint8_t encodedMessageType = static_cast<uint8_t>(messageType);
    std::vector<uint8_t> messageByteArray;

    messageByteArray.insert(messageByteArray.begin(), encodedMessageType);
    messageByteArray.insert(messageByteArray.end(), payload.begin(), payload.end());

    messageByteArray.resize(ProtocolConstants::DATA_FRAME_LENGTH, ProtocolConstants::PADDING_CHAR);     // Pad with null bytes

    return messageByteArray;
}

// Encode a vector of float
std::vector<uint8_t> ProtocolParser::encodeMessage(ProtocolConstants::RobotMessageType messageType, std::vector<float> payload) {
    uint8_t encodedMessageType = static_cast<uint8_t>(messageType);
    std::vector<uint8_t> messageByteArray;

    messageByteArray.resize(1 + payload.size() * sizeof(float));

    messageByteArray.insert(messageByteArray.begin(), encodedMessageType);
    memcpy(messageByteArray.data() + 1, payload.data(), payload.size() * sizeof(float));

    messageByteArray.resize(ProtocolConstants::DATA_FRAME_LENGTH, ProtocolConstants::PADDING_CHAR);     // Pad with null bytes

    return messageByteArray;
}

// Encode a vector of int
std::vector<uint8_t> ProtocolParser::encodeMessage(ProtocolConstants::RobotMessageType messageType, std::vector<int> payload) {
    uint8_t encodedMessageType = static_cast<uint8_t>(messageType);
    std::vector<uint8_t> messageByteArray;

    messageByteArray.resize(1 + payload.size() * sizeof(int));

    messageByteArray.insert(messageByteArray.begin(), encodedMessageType);
    memcpy(messageByteArray.data() + 1, payload.data(), payload.size() * sizeof(int));

    messageByteArray.resize(ProtocolConstants::DATA_FRAME_LENGTH, ProtocolConstants::PADDING_CHAR);     // Pad with null bytes

    return messageByteArray;
}

// Mainly for reading data from the Python socket server
void ProtocolParser::decodeMessage(
    std::vector<uint8_t> inputData,
    ProtocolConstants::RobotMessageType& messageType,
    ProtocolConstants::Responses& responseType,
    std::vector<uint8_t>& payload
    ) {

    int messageTypeIndex = static_cast<int>(ProtocolConstants::MessageIndex::messageType);
    int responseTypeIndex = static_cast<int>(ProtocolConstants::MessageIndex::messageResponse);

    messageType = static_cast<ProtocolConstants::RobotMessageType>(inputData[messageTypeIndex]);
    responseType = static_cast<ProtocolConstants::Responses>(inputData[responseTypeIndex]);

    removeTrailingNulls(inputData, messageType);

    inputData.erase(inputData.begin(), inputData.begin() + 2);  // Erase message type and index after extracting them
    payload = inputData;
}

void ProtocolParser::removeTrailingNulls(std::vector<uint8_t>& data, ProtocolConstants::RobotMessageType messageType) {
    size_t messageLength;
    messageLength = getMessageLength(messageType);

    data.erase(data.begin() + messageLength, data.end());
}

constexpr size_t ProtocolParser::getMessageLength(ProtocolConstants::RobotMessageType messageType) {
    using MsgT = ProtocolConstants::RobotMessageType;
    using MsgL = ProtocolConstants::MessageLength;
    switch(messageType) {
        case MsgT::Connect: return static_cast<size_t>(MsgL::Connect);
        case MsgT::Disconnect: return static_cast<size_t>(MsgL::Disconnect);
        case MsgT::Home: return static_cast<size_t>(MsgL::Home);
        case MsgT::Disable: return static_cast<size_t>(MsgL::Disable);
        case MsgT::ReadJointAngles: return static_cast<size_t>(MsgL::ReadJointAngles);
        case MsgT::UpdateEEPos: return static_cast<size_t>(MsgL::UpdateEEPos);
        case MsgT::SaveCurrentPosition: return static_cast<size_t>(MsgL::SaveCurrentPosition);
        default:
            std::cout << "No length found!";
            return static_cast<size_t>(0);
    }
}
