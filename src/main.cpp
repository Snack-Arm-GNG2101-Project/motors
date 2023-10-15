#include <Arduino.h>
#include <Stepper.h>

#define BAUD 9600
#define NUM_OF_CASES 5
#define STEPS_PER_REV 100
#define SIGNAL_BUTTON_PIN 34
#define MOTOR_SPEED 150

// for your motor

// initialize the stepper library on pins 8 through 11:
Stepper myStepper1(STEPS_PER_REV, 25,27,26,14);
Stepper myStepper2(STEPS_PER_REV,10,32,13,33);
Stepper myStepper3(STEPS_PER_REV, 21,19,22,23);

uint8_t motorswitch = 0;
uint8_t change;

void setup() {
    // set the speed at 70 rpm:
    myStepper1.setSpeed(MOTOR_SPEED);
    myStepper2.setSpeed(MOTOR_SPEED);
    myStepper3.setSpeed(MOTOR_SPEED);

    // initialize the serial port:
    Serial.begin(BAUD);
}

void loop() {
    // step one revolution  in one direction:
    Serial.println(change);
    change = analogRead(SIGNAL_BUTTON_PIN);

    if(change == 255 && motorswitch != NUM_OF_CASES){
        motorswitch += 1;
    }
    else if(change == 255 && motorswitch == NUM_OF_CASES){
        motorswitch = 0;
    }

    switch(motorswitch){
        case 0:
            myStepper1.step(STEPS_PER_REV);
            break;
        case 1:
            myStepper1.step(-STEPS_PER_REV);
            break;
        case 2:
            myStepper2.step(-STEPS_PER_REV);
            break;
        case 3:
            myStepper2.step(STEPS_PER_REV);
            break;
        case 4:
            myStepper3.step(-STEPS_PER_REV);
            break;
        case 5:
            myStepper3.step(STEPS_PER_REV);
            break;
        default:
            break;
    }
}
