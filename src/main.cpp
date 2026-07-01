#include <Arduino.h>
#include "chassis.hpp"
Motor left_motor(M1_IN1, M1_IN2, true);
Motor right_motor(M2_IN1, M2_IN2, true);
Chassis chassis(left_motor, right_motor);
void setup() { Serial.begin(115200); delay(500); left_motor.begin(0); right_motor.begin(2); }
void loop() { chassis.chassisTask(); }
