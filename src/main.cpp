#include <Arduino.h>

#define BAUD 9600
#define WHEEL_DIAMETER_INCHES 3.0f
#define X_MOTOR_RPM 30.0f // MAX 30 RPM
#define Z_MOTOR_RPM 15.0f // MAX 30 RPM
#define X_DISTANCE_INCHES 27.0f
#define Z_PICK_UP_DISTANCE_INCHES 6.0f
#define Z_DROP_OFF_DISTANCE_INCHES 2.0f
#define RED_BUTTON_PIN 13
#define X_MOTOR_PIN 14
#define Z_MOTOR_PIN 15

uint32_t timeToClearDistanceMS;
uint8_t motorState;
uint32_t startTimeMS, endTimeMS;

float rpmToTimeMS(float distanceInches, float rpm) {
    // v = circumference (inches) / min
    // v = d/t
    // t = d/v
    float wheelCircumference = 2 * (WHEEL_DIAMETER_INCHES / 2) * PI;
    float angularVelocity = wheelCircumference * rpm;
    return (distanceInches / angularVelocity) * 60000;
}

// this function controls behavior of the arm during the entire process of picking up the snack and bringing it back
void pickUpSnack() {
    uint32_t pickUpTimeMS = floor(rpmToTimeMS(Z_PICK_UP_DISTANCE_INCHES, Z_MOTOR_RPM));
    uint32_t dropOffTimeMS = floor(rpmToTimeMS(Z_DROP_OFF_DISTANCE_INCHES, Z_MOTOR_RPM));

    Serial.println("TURNING THE MOTOR TO GO DOWN...");
    delay(pickUpTimeMS);
    Serial.println("done.");

    Serial.println("TURNING THE MOTOR TO GO UP...");
    delay(pickUpTimeMS);
    Serial.println("done.");

    Serial.println("TURNING THE MOTOR TO GO BACK...");
    delay(timeToClearDistanceMS);
    Serial.println("done.");

    Serial.println("TURNING THE MOTOR TO DROP OFF THE SNACK...");
    delay(dropOffTimeMS);
    Serial.println("done.");

    Serial.println("BRINGING THE MOTOR BACK UP...");
    delay(dropOffTimeMS);
    Serial.println("done.");
}

void setup() {
    Serial.begin(BAUD);
    pinMode(RED_BUTTON_PIN, INPUT);
    pinMode(X_MOTOR_PIN, OUTPUT);
    pinMode(Z_MOTOR_PIN, OUTPUT);
    timeToClearDistanceMS = floor(rpmToTimeMS(X_DISTANCE_INCHES, X_MOTOR_RPM));
    motorState = 0;
    startTimeMS = millis();
}

void loop() {
    int buttonPressed = digitalRead(RED_BUTTON_PIN);
    if (buttonPressed) {
        pickUpSnack();
        motorState = 0;
        startTimeMS = millis();
    }

    if (motorState == 0) {
        Serial.println("Turning the motor left...");
    }
    else if (motorState == 1) {
        Serial.println("Turning the motor right...");
    }

    endTimeMS = millis();
    if (endTimeMS - startTimeMS >= timeToClearDistanceMS) {
        motorState = !motorState;
        startTimeMS = millis();
    }
}
