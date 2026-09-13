// This class was made by Gemini, as a test to see how using perlin noise
// works for making random pathfinding for the robot

#include <iostream>
#include <cmath>
#include <Arduino.h>

class StepperNoiseDriver {
private:
    double time_linear;
    double time_angular;
    double step_linear;
    double step_angular;

    // Smooth continuous noise generator
    double noise(double t) {
        return (std::sin(t) + std::sin(t * 2.3) + std::sin(t * 5.7)) / 3.0;
    }

public:
    // Outputs for your hardware controller
    int left_steps;
    int right_steps;

    // Physical Constants (Adjust these to match your specific vehicle build)
    const double wheel_base   = 0.12;   // Distance between left/right wheels (meters)
    const double wheel_radius = 0.068;   // Radius of the wheel (meters)
    const int steps_per_rev   = 200;    // Standard stepper steps per full 360 rotation
    const double dt           = 1;    // Time step duration (seconds)

    // Limits
    const double max_linear_velocity  = 0.5;  // Max forward speed (m/s)
    const double max_angular_velocity = 2.0;  // Max turn rate (rad/s)

    StepperNoiseDriver() {
        left_steps = 0;
        right_steps = 0;

        time_linear = 0.0;
        time_angular = 5000.0; // Independent seed offset

        step_linear = 0.02;  // Speed variations change gently
        step_angular = 0.08; // Turning changes more rapidly
    }

    void update() {
        // 1. Fetch smooth noise values (-1.0 to 1.0)
        double n_lin = noise(time_linear);
        double n_ang = noise(time_angular);

        if (random(0, 10) == 0 && n_lin > 0) n_lin = -n_lin;

        // 2. Map noise to Target Velocities
        double target_linear  = n_lin * max_linear_velocity; 
        double target_angular = n_ang * max_angular_velocity;

        // 3. Differential Drive Kinematics
        // Calculate the linear velocity required for each individual wheel
        double v_left  = target_linear - (target_angular * wheel_base / 2.0);
        double v_right = target_linear + (target_angular * wheel_base / 2.0);

        // 4. Convert Linear Wheel Velocity to Angular Velocity (rad/s)
        double omega_left  = v_left / wheel_radius;
        double omega_right = v_right / wheel_radius;

        // 5. Convert Angular Velocity to Step Pulses for this specific 'dt' interval
        // Formula: Steps = (rad/s * dt) * (steps_per_revolution / 2 * pi)
        double steps_factor = steps_per_rev / (2.0 * M_PI);
        
        left_steps  = static_cast<int>(std::round(omega_left * dt * steps_factor));
        right_steps = static_cast<int>(std::round(omega_right * dt * steps_factor));

        // 6. Advance timelines
        time_linear  += step_linear;
        time_angular += step_angular;
    }
};