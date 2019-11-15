#include "hardware_config.hpp"
#include <stdint.h>

// If 1, prints debug information to the Arduino Serial Monitor, if 0, doesn't
#define DEBUG_ENABLED 1

enum FsmState {Stopped, Fwd, Rev, Left, Right};
FsmState cur_state;

void setup() {
    // Fix these pins
    pinMode(pins::motorR1, OUTPUT);
    pinMode(pins::motorR2, OUTPUT);
    pinMode(pins::motorL1, OUTPUT);
    pinMode(pins::motorL2, OUTPUT);
    
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
            digitalWrite(pins::motorR1, HIGH);
            digitalWrite(pins::motorR2, HIGH);
            digitalWrite(pins::motorL1, HIGH);
            digitalWrite(pins::motorL2, HIGH);
            // If edge detected in front 
            // Reverse?
            // Else if opponent detected in front
            // Forward
            break;
        case Fwd:
            digitalWrite(pins::motorR1, HIGH);
            digitalWrite(pins::motorR2, LOW);
            digitalWrite(pins::motorL1, HIGH);
            digitalWrite(pins::motorL2, LOW);
            // Switching Logic
            break;
        case Rev:
            digitalWrite(pins::motorR1, LOW);
            digitalWrite(pins::motorR2, HIGH);
            digitalWrite(pins::motorL1, LOW);
            digitalWrite(pins::motorL2, HIGH);
            // Switching Logic
            break;
        case Left:
            digitalWrite(pins::motorR1, HIGH);
            digitalWrite(pins::motorR2, LOW);
            digitalWrite(pins::motorL1, LOW);
            digitalWrite(pins::motorL2, HIGH);
            // Switching Logic
            break;
        case Right:
            digitalWrite(pins::motorR1, LOW);
            digitalWrite(pins::motorR2, HIGH);
            digitalWrite(pins::motorL1, HIGH);
            digitalWrite(pins::motorL2, LOW);
            // Switching Logic
            break;
    }
    delay(200);
}
