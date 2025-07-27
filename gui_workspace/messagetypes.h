#ifndef MESSAGETYPES_H
#define MESSAGETYPES_H

#include "protocolconstants.h"
#include <variant>
#include <stdint.h>
#include <vector>

#include <QByteArray>

struct Connect {
    ProtocolConstants::Responses response;
};

struct Disconnect {
    ProtocolConstants::Responses response;
};

struct Home {
    ProtocolConstants::Responses response;
    std::vector<float> jointAngles = std::vector<float>(5);
    std::vector<float> xyzPosition = std::vector<float>(3);
};

struct JointAngles {
    std::vector<float> angles = std::vector<float>(5);
};

struct XYZPosition {
    std::vector<float> coordinates = std::vector<float>(3);
};

struct JointAnglesAndPosition {
    JointAngles angles;
    XYZPosition coordinates;
};

using DataVariant = std::variant<
    Connect,
    Disconnect,
    Home,
    JointAngles,
    XYZPosition,
    JointAnglesAndPosition
>;

#endif // MESSAGETYPES_H
