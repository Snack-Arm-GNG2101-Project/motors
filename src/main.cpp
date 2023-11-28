#include <Arduino.h>
#include <Stepper.h>

#define BAUD 9600
#define WHEEL_DIAMETER_INCHES 3.0f
#define X_MOTOR_RPM 40.0f // MAX 30 RPM
#define Z_MOTOR_RPM 40.0f // MAX 30 RPM
#define X_DISTANCE_INCHES 27.0f
#define Z_PICK_UP_DISTANCE_INCHES 6.0f
#define Z_DROP_OFF_DISTANCE_INCHES 2.0f
#define RED_BUTTON_PIN 13
#define X_MOTOR_PIN_1 2
#define X_MOTOR_PIN_2 0
#define X_MOTOR_PIN_3 4
#define X_MOTOR_PIN_4 9
#define Z_MOTOR_PIN_1 10
#define Z_MOTOR_PIN_2 5
#define Z_MOTOR_PIN_3 18
#define Z_MOTOR_PIN_4 23

#define STEPS_PER_REV 130 // mess with this later

// defining the stepper motors
Stepper xMotor(STEPS_PER_REV, X_MOTOR_PIN_1, X_MOTOR_PIN_2, X_MOTOR_PIN_3, X_MOTOR_PIN_4);
Stepper zMotor(STEPS_PER_REV,Z_MOTOR_PIN_1, Z_MOTOR_PIN_2, Z_MOTOR_PIN_3, Z_MOTOR_PIN_4);

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
    zMotor.step(STEPS_PER_REV);
    delay(pickUpTimeMS);
    Serial.println("done.");

    Serial.println("TURNING THE MOTOR TO GO UP...");
    zMotor.step(-STEPS_PER_REV);
    delay(pickUpTimeMS);
    Serial.println("done.");

    Serial.println("TURNING THE MOTOR TO GO BACK...");
    xMotor.step(-STEPS_PER_REV);
    delay(timeToClearDistanceMS);
    Serial.println("done.");

    Serial.println("TURNING THE MOTOR TO DROP OFF THE SNACK...");
    zMotor.step(STEPS_PER_REV);
    delay(dropOffTimeMS);
    Serial.println("done.");
}

void setup() {
    Serial.begin(BAUD);
    Serial.println("!! GOD DID !!");
    pinMode(RED_BUTTON_PIN, INPUT);
    xMotor.setSpeed(X_MOTOR_RPM);
    zMotor.setSpeed(Z_MOTOR_RPM);

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
        xMotor.step(STEPS_PER_REV);
        Serial.println("Turning the motor left...");
    }
    else if (motorState == 1) {
        xMotor.step(-STEPS_PER_REV);
        Serial.println("Turning the motor right...");
    }

    endTimeMS = millis();
    if (endTimeMS - startTimeMS >= timeToClearDistanceMS) {
        motorState = !motorState;
        startTimeMS = millis();
    }
}
