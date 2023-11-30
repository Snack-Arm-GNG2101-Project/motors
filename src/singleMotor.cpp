#include <Arduino.h>
#include <Stepper.h>

#define BAUD 9600
#define STEPS_PER_REV 130
#define RPM 30.0

Stepper stepper(STEPS_PER_REV, 2, 15, 13, 12);

void setup() {
    Serial.begin(BAUD);
    stepper.setSpeed(RPM);
}

void loop() {
    stepper.step(1);
}