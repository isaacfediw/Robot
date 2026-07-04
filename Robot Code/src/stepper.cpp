#include "stepper.h"
#include <Arduino.h>

#define STEP 0
#define DIR  1
#define EN   2

#define BASE_STEP_INTERVAL 1000 // slowest step interval

stepper::stepper(int stepper_pins[3]) {
    start = micros();
    lastStepMicros = 0; 
    stepState = false;
    busy = false;

    for (int i = 0; i < 3; i++) {
        pins[i] = stepper_pins[i];
        pinMode(pins[i], OUTPUT);
    }
}

void stepper::moveStepper(int dir, int steps) {
    if (busy) return;

    digitalWrite(pins[DIR], dir);
    digitalWrite(pins[EN], 0);
    stepsRemaining = steps * 8;

    acceleration = (int) ((float) BASE_STEP_INTERVAL / (float) (stepsRemaining/2 + 1));
    stepInterval = (acceleration > 0) ? BASE_STEP_INTERVAL : BASE_STEP_INTERVAL / 2;
    intervalDirection = -1;

    busy = true;
}

bool stepper::isBusy() {return busy;}

void stepper::stepperLoop() {
    start = micros();
    if (start - lastStepMicros >= stepInterval) {
        lastStepMicros = micros();
        stepState = !stepState;

        if (stepsRemaining > 0) {
            digitalWrite(pins[STEP], stepState);

            if (!stepState) {
                stepsRemaining --;

                if (stepsRemaining <= 0) {
                    digitalWrite(pins[EN], 1);
                    busy = false;
                }
            }
        }
 
        if (stepState) {
            stepInterval = stepInterval + intervalDirection * acceleration;
            if (stepInterval <= 200 || stepInterval >= BASE_STEP_INTERVAL) intervalDirection *= -1;
        }   
    }
}