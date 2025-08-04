#ifndef PROTOCOLPARSER_H
#define PROTOCOLPARSER_H

#include <iostream>
#include <vector>
#include <cstdint>
#include <string>
#include <variant>

#include <QByteArray>

#include "protocolconstants.h"
#include "messagetypes.h"

class ProtocolParser {

/* TODO:
 * - Define primary functions (encode/decode)
 * -
 * -
 */

public:
    ProtocolParser();

    std::vector<uint8_t> encodeMessage(
        ProtocolConstants::RobotMessageType messageType
        // std::optional<uint8_t> payload = std::nullopt
    );

    std::vector<uint8_t> encodeMessage(
        ProtocolConstants::RobotMessageType messageType,
        std::vector<uint8_t> payload
    );

    std::vector<uint8_t> encodeMessage(
        ProtocolConstants::RobotMessageType messageType,
        std::vector<float> payload
    );

    std::vector<uint8_t> encodeMessage(
        ProtocolConstants::RobotMessageType messageType,
        std::vector<int> payload
    );

    void decodeMessage(
        std::vector<uint8_t> data,
        ProtocolConstants::RobotMessageType& messageType,
        ProtocolConstants::Responses& responseType,
        std::vector<uint8_t>& payload
    );

    void removeTrailingNulls(std::vector<uint8_t>& data, ProtocolConstants::RobotMessageType messageType);
    constexpr size_t getMessageLength(ProtocolConstants::RobotMessageType messageType);

};

#endif // PROTOCOLPARSER_H
