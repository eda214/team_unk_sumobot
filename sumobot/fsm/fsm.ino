#include "hardware_config.hpp"
#include <stdint.h>

// If 1, prints debug information to the Arduino Serial Monitor, if 0, doesn't
#define DEBUG_ENABLED 1

// Threshold for whether refl sensor detects a line (just made up a number for now)
#define REF_THRESHOLD 500

enum FsmState {Stopped, Fwd, Rev, Left, Right};
FsmState cur_state;
bool edgeInFront();
bool edgeInRear();
bool opntInFront();

void setup() {
    // Change pin numbers in hardware_config.hpp
    pinMode(pins::motorR1, OUTPUT);
    pinMode(pins::motorR2, OUTPUT);
    pinMode(pins::motorL1, OUTPUT);
    pinMode(pins::motorL2, OUTPUT);
    
    pinMode(pins::refFront, INPUT);
    pinMode(pins::refRear, INPUT);

    // Starting at stopped
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
            if(edgeInFront()) {
              cur_state = Rev;
            }
            else if (opntInFront()) {
              cur_state = Fwd;
            }
            // What else?
            break;
        case Fwd:
            digitalWrite(pins::motorR1, HIGH);
            digitalWrite(pins::motorR2, LOW);
            digitalWrite(pins::motorL1, HIGH);
            digitalWrite(pins::motorL2, LOW);
            // Switching Logic
            if (edgeInFront()) {
              cur_state = Rev;
            }
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
            //
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

bool edgeInFront() {
  return (analogRead(pins::refFront) < REF_THRESHOLD);
}

bool edgeInRear() {
  return (analogRead(pins::refRear) < REF_THRESHOLD);
}

// Should return true if opponent detected in front, false otherwise
bool opntInFront() {
  return false;
}
