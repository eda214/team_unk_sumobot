#include "hardware_config.hpp"
#include <stdint.h>

// If 1, prints debug information to the Arduino Serial Monitor, if 0, doesn't
#define DEBUG_ENABLED 1

enum FsmState {Stopped, Fwd, Rev, Left, Right};
FsmState cur_state;

void setup() {
    // Fix these pins
    pinMode(pins::motorA1, OUTPUT);
    pinMode(pins::motorA2, OUTPUT);
    pinMode(pins::motorB1, OUTPUT);
    pinMode(pins::motorB2, OUTPUT);
    
    pinMode(pins::ref1, INPUT);
    pinMode(pins::ref2, INPUT);

    // Counter starts at 0
    cur_state = Stopped;

    #if DEBUG_ENABLED
        Serial.begin(9600);
        Serial.println("Initialization complete.");
    #endif
}

void loop() {
    #if DEBUG_ENABLED
        Serial.print("State: ");
        Serial.println(cur_state);
    #endif

    // Read the two buttons and keep track of their state for this cycle
    switch (cur_state) {
        case Stopped:
            digitalWrite(pins::motorA1, HIGH);
            digitalWrite(pins::motorA2, HIGH);
            digitalWrite(pins::motorB1, HIGH);
            digitalWrite(pins::motorB2, HIGH);
            // If edge detected in front 
            // Reverse?
            // Else if opponent detected in front
            // Forward
            break;
        case Fwd:
            digitalWrite(pins::motorA1, HIGH);
            digitalWrite(pins::motorA2, LOW);
            digitalWrite(pins::motorB1, HIGH);
            digitalWrite(pins::motorB2, LOW);
            // Switching Logic
        case Rev:
            digitalWrite(pins::motorA1, LOW);
            digitalWrite(pins::motorA2, HIGH);
            digitalWrite(pins::motorB1, LOW);
            digitalWrite(pins::motorB2, HIGH);
            // Switching Logic
    }
    delay(200);
}
