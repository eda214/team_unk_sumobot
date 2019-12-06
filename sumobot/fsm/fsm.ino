#include "hardware_config.hpp"
#include <stdint.h>

#include <VL53L0X.h>

#include <Wire.h>
#include <WireIMXRT.h>
#include <WireKinetis.h>

// If 1, prints debug information to the Arduino Serial Monitor, if 0, doesn't
#define DEBUG_ENABLED 1

// Constants
// Threshold for whether refl sensor detects a line (placeholder number for now)
#define REF_THRESHOLD 0
// Threshold for TOF (also placeholder)
#define TOF_THRESHOLD 100
// PWM values for motors
#define HIGH_PWM 255
#define LOW_PWM 127

// States
enum FsmState {Stopped, Fwd, Rev, FwdHigh, RevHigh, Left, Right};
FsmState cur_state;
FsmState search_dir; //Tells which way to turn if nothing else to do

// I^2C stuff
void init_sensor_(uint8_t index);

const int num_tof_sensors = 2;
VL53L0X* sensors_[num_tof_sensors] = {new VL53L0X, new VL53L0X};;
String sensor_names_[num_tof_sensors] = {"left", "right"};
int sensor_pins_[num_tof_sensors] = {pins::XSHUT1, pins::XSHUT2};

// Reflectance sensor readings
bool edgeFL();
bool edgeFR();
bool edgeRL();
bool edgeRR();

bool tofFront();
bool tofRear();

void setup() {
    // Change pin numbers in hardware_config.hpp
    pinMode(pins::motorR1, OUTPUT);
    pinMode(pins::motorR2, OUTPUT);
    pinMode(pins::motorL1, OUTPUT);
    pinMode(pins::motorL2, OUTPUT);

    // Need to check if we actually need these
    pinMode(pins::refFL, INPUT);
    pinMode(pins::refFR, INPUT);
    pinMode(pins::refRL, INPUT);
    pinMode(pins::refRR, INPUT);

    // Starting at stopped
    cur_state = Stopped;

    // We'll turn left if nothing else to do at first
    search_dir = Left;

    for (uint8_t i = 0; i < num_tof_sensors; i++) {
        pinMode(sensor_pins_[i], OUTPUT);
        digitalWrite(sensor_pins_[i], LOW);
    }
  
    delay(50);
    Wire.begin();
  
    // Set sensor addresses
    for (uint8_t i = 0; i < num_tof_sensors; i++) {
        digitalWrite(sensor_pins_[i], HIGH);
        delay(50);
        sensors_[i]->setAddress(2 * i);
        // Uncomment to debug addresses of sensors
        // Serial.println(sensors[i]->readReg(0x212));
    }
    delay(50);
  
    // Initializes sensors
    for (uint8_t i = 0; i < num_tof_sensors; i++) {
        initSensor_(i);
    }

    #if DEBUG_ENABLED
        Serial.begin(9600);
        Serial.println("Initialization complete.");
    #endif

    // 5-second delay
    delay(5000);
}

void loop() {
    #if DEBUG_ENABLED
        Serial.print("State: ");
        Serial.println(cur_state);
        Serial.print("IR Front Left: ");
        Serial.println(analogRead(pins::refFL));
        Serial.print("IR Front Right: ");
        Serial.println(analogRead(pins::refFR));
        Serial.print("IR Rear Left: ");
        Serial.println(analogRead(pins::refRL));
        Serial.print("IR Rear Right: ");
        Serial.println(analogRead(pins::refRR));
        Serial.print("TOF Front: ");
        Serial.println(getTOF(0));
        Serial.print("TOF Rear: ");
        Serial.println(getTOF(1));
    #endif

    // Read the two buttons and keep track of their state for this cycle
    switch (cur_state) {
        case Stopped:
            digitalWrite(pins::motorR1, HIGH);
            digitalWrite(pins::motorR2, HIGH);
            digitalWrite(pins::motorL1, HIGH);
            digitalWrite(pins::motorL2, HIGH);
            // Opponent in front
            if (opntInFront()) {
              cur_state = FwdHigh;
            }
            // Edge in front -> reverse 
            else if(edgeFL() && edgeFR()) {
              cur_state = Rev;
            }
            // Edge behind -> forward
            else if (edgeRL() && edgeRR()) {
              cur_state = Fwd;
            }
            // Edge on left -> turn right
            else if (edgeFL() && edgeRL()) {
              search_dir = Right;
              cur_state = Right;
            }
            // Edge on right -> turn left
            else if (edgeFR() && edgeRR()) {
              search_dir = Left;
              cur_state = Left;
            }
            // Edge in front corner -> turn away from edge
            // NEED BETTER RESPONSE HERE
            // Possibly more specific kinds of turns?
            else if (edgeFL()) {
              search_dir = Right;
              cur_state = Right;
            }
            else if (edgeFR()) {
              search_dir = Left;
              cur_state = Left;
            }
            // Edge in rear corner -> turn away from edge
            // Probably fine in most cases, but test
            else if (edgeRL()) {
              search_dir = Right;
              cur_state = Right;
            }
            else if (edgeRR()) {
              search_dir = Left;
              cur_state = Left;
            }
            else {
              cur_state = search_dir;
            }
            // What else?
            break;
        case Fwd:
            digitalWrite(pins::motorR1, HIGH);
            digitalWrite(pins::motorR2, LOW);
            analogWrite(pins::motorRPWM, LOW_PWM);
            digitalWrite(pins::motorL1, HIGH);
            digitalWrite(pins::motorL2, LOW);
            analogWrite(pins::motorLPWM, LOW_PWM);
            // Switching Logic
            if (opntInFront()) {
              cur_state = FwdHigh;
            }
            // If driving parallel to edge, turn away from it
            else if (edgeFL() && edgeRL()) {
              cur_state = Right;
            }
            else if (edgeFR() && edgeRR()) {
              cur_state = Left;
            }
            else if (edgeFL() || edgeFR()) {
              cur_state = Stopped;
            }
            break;
        case Rev:
            digitalWrite(pins::motorR1, LOW);
            digitalWrite(pins::motorR2, HIGH);
            analogWrite(pins::motorRPWM, LOW_PWM);
            digitalWrite(pins::motorL1, LOW);
            digitalWrite(pins::motorL2, HIGH);
            analogWrite(pins::motorLPWM, LOW_PWM);
            // Switching Logic
            if (opntInFront()) {
              cur_state = FwdHigh;
            }
            // If reversing and edge is detected in front, reverse harder
            else if (edgeFL() && edgeFR()) {
              cur_state = RevHigh;
            }
            // If driving parallel to edge, turn away from it
            else if (edgeFL() && edgeRL()) {
              search_dir = Right;
              cur_state = Right;
            }
            else if (edgeFR() && edgeRR()) {
              search_dir = Left;
              cur_state = Left;
            }
            // Fix this case
            else if (edgeRL() || edgeRR()) {
              cur_state = Stopped;
            }
            break;
        case FwdHigh:
            digitalWrite(pins::motorR1, HIGH);
            digitalWrite(pins::motorR2, LOW);
            analogWrite(pins::motorRPWM, HIGH_PWM);
            digitalWrite(pins::motorL1, HIGH);
            digitalWrite(pins::motorL2, LOW);
            analogWrite(pins::motorLPWM, HIGH_PWM);
            // Switching Logic
            if (!opntInFront()) {
              cur_state = Fwd;
            }
            break;
        case RevHigh:
            digitalWrite(pins::motorR1, LOW);
            digitalWrite(pins::motorR2, HIGH);
            analogWrite(pins::motorRPWM, HIGH_PWM);
            digitalWrite(pins::motorL1, LOW);
            digitalWrite(pins::motorL2, HIGH);
            analogWrite(pins::motorLPWM, HIGH_PWM);
            // Switching Logic
            // Is this criterion for switching out too strict?
            if (!edgeFL() && !edgeRL()) {
              cur_state = Rev;
            }
            break;
        case Left:
            digitalWrite(pins::motorR1, HIGH);
            digitalWrite(pins::motorR2, LOW);
            analogWrite(pins::motorRPWM, LOW_PWM);
            digitalWrite(pins::motorL1, LOW);
            digitalWrite(pins::motorL2, HIGH);
            analogWrite(pins::motorLPWM, LOW_PWM);
            // Switching Logic
            if (opntInFront()) {
              cur_state = FwdHigh;
            }
            else if (edgeFL() || edgeRL()) {
              search_dir = Right;
              cur_state = Stopped;
            }
            break;
        case Right:
            digitalWrite(pins::motorR1, LOW);
            digitalWrite(pins::motorR2, HIGH);
            analogWrite(pins::motorRPWM, LOW_PWM);
            digitalWrite(pins::motorL1, HIGH);
            digitalWrite(pins::motorL2, LOW);
            analogWrite(pins::motorLPWM, LOW_PWM);
            // Switching Logic
            if (opntInFront()) {
              cur_state = FwdHigh;
            }
            else if (edgeFR() || edgeRR()) {
              search_dir = Left;
              cur_state = Stopped;
            }
            break;
    }
    delay(200);
}


bool edgeFL() {
  return (analogRead(pins::refFL) < REF_THRESHOLD);
}

bool edgeFR() {
  return (analogRead(pins::refFR) < REF_THRESHOLD);
}

bool edgeRL() {
  return (analogRead(pins::refRL) < REF_THRESHOLD);
}

bool edgeRR() {
  return (analogRead(pins::refRR) < REF_THRESHOLD);
}

// Should return true if something detected, false otherwise
uint16_t getTOF(int i) {
  return sensors_[i]->readRangeContinuousMillimeters();
}

bool tofFront() {
  return (getTOF(0) < TOF_THRESHOLD);
}

bool tofRear() {
  return (getTOF(1) < TOF_THRESHOLD);
}

bool opntInFront() {
  return (tofFront());
}

bool opntInRear() {
  return (tofRear());
}

void initSensor_(uint8_t index) {
   VL53L0X* sensor = sensors_[index];
   sensor->init();
   sensor->setTimeout(500);
   sensor->startContinuous();
   Serial.print(sensor_names_[index]);
   Serial.println(" online.");
   delay(200);
}
