#ifndef CLIENTTYPES_H
#define CLIENTTYPES_H

#include "protocolconstants.h"
#include <variant>
#include <stdint.h>
#include <vector>




using SignalDataVariant = std::variant<
    JointAngles
>;


#endif // CLIENTTYPES_H
