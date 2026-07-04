#pragma once

class stepper {
    private:
        long stepsRemaining;

        // step, dir, en
        int pins[3];
        bool busy;

        unsigned long lastStepMicros;
        unsigned long start;
        int stepInterval, acceleration, intervalDirection;
        bool stepState;

    public:
        // class constructor
        // step, dir, en
        stepper(int pins[3]);

        // enables the stepper for moving in stepperLoop()
        void moveStepper(int dir, int steps);

        // to move the stepper this must be called every iteration of loop() inside the main file
        void stepperLoop();

        // checks if the stepper is currently busy
        bool isBusy();
};