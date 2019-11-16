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

    // reflectance sensor pins
    const uint8_t refFront = 22;
    const uint8_t refRear = 23;

    // tof sensor pins
    const uint8_t scl = 19;
    const uint8_t sda = 18;
    const uint8_t xshut1 = 2; // Better names for these?
    const uint8_t xshut2 = 3;
}   // namespace pins
