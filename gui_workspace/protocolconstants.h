#pragma once
#ifndef PROTOCOLCONSTANTS_H
#define PROTOCOLCONSTANTS_H

#include <stdint.h>
#include <stddef.h>
#include <variant>

// Define an enum class (user-defined type for giving names to integer constants)
namespace ProtocolConstants {
    enum class RobotMessageType : uint8_t {
        Connect = 0x01,                 // 1
        Disconnect = 0x02,              // 2
        Home = 0x0A,                    // 10
        Disable = 0x0B,                 // 11
        ReadJointAngles = 0x0C,         // 12
        UpdateEEPos = 0x0D,             // 13
        UpdateEEOrientation = 0x0E,     // 14
        MoveX = 0x14,                   // 20
        MoveY = 0x15,                   // 21
        MoveZ = 0x16,                   // 22
        MoveJ1 = 0x1E,                  // 30
        MoveJ2 = 0x1F,                  // 31
        MoveJ3 = 0x20,                  // 32
        MoveJ4 = 0x21,                  // 33
        MoveJ5 = 0x22,                  // 34
        SaveCurrentPosition = 0x28,     // 40
        SaveSequence = 0x29,            // 41
        GoToPosition = 0x2A,            // 42
        PlayBack = 0x2B,                // 43
        PlaySequence = 0x2C,            // 44
    };

    enum class MessageLength : size_t {
        Connect = 2,
        Disconnect = 2,
        Home = 34,
        Disable = 2,
        ReadJointAngles = 22,
        UpdateEEPos = 14,
        UpdateEEOrientation = 14,
        MoveX = 3,
        MoveY = 3,
        MoveZ = 3,
        MoveJ1 = 3,
        MoveJ2 = 3,
        MoveJ3 = 3,
        MoveJ4 = 3,
        MoveJ5 = 3,
        SaveCurrentPosition = 2,
        SaveSequence = 32,  // Message type, response, sequence ID, max 29 positions
        GoToPosition = 3,
        PlayBack = 32,       // Message type, response, max 30 positions
        PlaySequence = 3,
    };

    enum class MessageIndex : uint8_t {
        messageType = 0,
        messageResponse = 1,
        payloadStart = 2
    };

    enum class Responses : uint8_t {
        OK = 0x64,         // 100
        ERROR = 0x65       // 101
    };

    constexpr int DATA_FRAME_LENGTH = 64;
    constexpr uint8_t PADDING_CHAR = 0x00;
}

#endif // PROTOCOLCONSTANTS_H
