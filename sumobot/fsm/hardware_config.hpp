/**
 * hardware_config.hpp
 * Contains hardware constants for the two-bit counter
 */
#include <stdint.h>
#pragma once

namespace pins {
    // motor driver pins
    const uint8_t motorR1 = 21;
    const uint8_t motorR2 = 20;
    const uint8_t motorRPWM = 17;
    const uint8_t motorL1 = 15;
    const uint8_t motorL2 = 14;
    const uint8_t motorLPWM = 16;
}   // namespace pins
