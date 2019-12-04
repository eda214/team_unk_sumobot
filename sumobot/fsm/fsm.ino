#include "hardware_config.hpp"
#include <stdint.h>

#include <VL53L0X.h>

#include <Wire.h>
#include <WireIMXRT.h>
#include <WireKinetis.h>

// If 1, prints debug information to the Arduino Serial Monitor, if 0, doesn't
#define DEBUG_ENABLED 1

// Threshold for whether refl sensor detects a line (placeholder number for now)
#define REF_THRESHOLD 0

enum FsmState {Stopped, Fwd, Rev, Left, Right};
FsmState cur_state;

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

bool opntInFront();

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
            //if(edgeInFront()) {
            //  cur_state = Rev;
            //}
            //else if (opntInFront()) {
            //  cur_state = Fwd;
            //}
            // What else?
            break;
        case Fwd:
            digitalWrite(pins::motorR1, HIGH);
            digitalWrite(pins::motorR2, LOW);
            digitalWrite(pins::motorL1, HIGH);
            digitalWrite(pins::motorL2, LOW);
            // Switching Logic
            //if (edgeInFront()) {
            //  cur_state = Rev;
            //}
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

// Should return true if opponent detected in front, false otherwise
bool opntInFront() {
  return false;
}

void initSensor_(uint8_t index) {
   VL53L0X* sensor = sensors_[index];
   sensor->init();
   sensor->setTimeout(500);
   Serial.print(sensor_names_[index]);
   Serial.println(" online.");
   delay(200);
}

/*
 * sensors_ = {new VL53L0X, new VL53L0X, new VL53L0X};
sensor_names_ = {“left”, “front”, right”};
sensor_pins_= {XSHUT1, XSHUT2, XSHUT3};
for (uint8_t i = 0; i < sensor_pins_.size(); i++) {
       pinMode(sensor_pins_[i], OUTPUT);
       digitalWrite(sensor_pins_[i], LOW);
   }

   delay(50);
   Wire.begin();

   // Set sensor addresses
   for (uint8_t i = 0; i < sensor_pins_.size(); i++) {
       digitalWrite(sensor_pins_[i], HIGH);
       delay(50);
       sensors_[i]->setAddress(2 * i);
       // Uncomment to debug addresses of sensors
       // Serial.println(sensors[i]->readReg(0x212));
   }
   delay(50);

   // Initializes sensors
   for (uint8_t i = 0; i < sensor_pins_.size(); i++) {
       initSensor_(i);
 }

initSensor_(uint8_t index) {
   VL53L0X* sensor = sensors_[index];
   sensor->init();
   sensor->setTimeout(500);
   Serial.print(sensor_names_[index]);
   Serial.println(” online.“);
   delay(200);
}
 */
