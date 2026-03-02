#ifndef CHUTE_H
#define CHUTE_H

#include "definitions.h"

// State for proportional navigation (tracks bearing history)
struct PNState
{
    float prev_bearing;
    TickType_t prev_time;
    bool initialized;
};

// Compute bearing (degrees) from current position to target
// Returns angle in [-180, 180] where 0 = North, 90 = East
float compute_bearing(GPSData current, GPSData target);

// Normalize angle to [-180, 180] range
float normalize_angle(float angle);

// Proportional navigation controller
// Returns desired turn rate based on bearing rate to target
float proportional_navigation(GPSData current, GPSData target, PNState* state);

// Compute control output: desired heading correction (degrees)
// Combines PN guidance with current heading to produce steering command
float compute_steering(GPSData current, GPSData target, Orient current_heading, PNState* state);

// Motor command output
struct MotorCommand
{
    int8_t direction;  // -1 = left, 0 = stop, +1 = right
    uint8_t pwm;       // 0-255 motor speed
};

// Convert steering angle to motor command
MotorCommand steering_to_motor(float steering);

void ParachuteTask(void* params);

#endif
