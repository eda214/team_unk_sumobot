/**
 * hardware_config.hpp
 * Contains hardware constants for the two-bit counter
 */
#include <stdint.h>
#pragma once

namespace pins {
    // motor driver pins
    const uint8_t motorR1 = 10;
    const uint8_t motorR2 = 9;
    const uint8_t motorRPWM = 17;
    const uint8_t motorL1 = 14;
    const uint8_t motorL2 = 15;
    const uint8_t motorLPWM = 16;

    // reflectance sensor pins
    // Front left, front right, rear left, rear right
    const uint8_t refFL = 22;
    const uint8_t refFR = 23;
    const uint8_t refRL = 24; //placeholder number
    const uint8_t refRR = 25; //^^^

    // tof sensor pins
    const uint8_t scl = 19;
    const uint8_t sda = 18;
    const uint8_t XSHUT1 = 2; // Better names for these?
    const uint8_t XSHUT2 = 3;
}   // namespace pins
