#include "chute.h"
#include "config.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#ifndef DEG_TO_RAD
#define DEG_TO_RAD (M_PI / 180.0f)
#endif

#ifndef RAD_TO_DEG
#define RAD_TO_DEG (180.0f / M_PI)
#endif

float compute_bearing(GPSData current, GPSData target)
{
    // Convert to radians for trig
    float lat1 = current.lat * DEG_TO_RAD;
    float lat2 = target.lat * DEG_TO_RAD;
    float dlon = (target.lon - current.lon) * DEG_TO_RAD;

    // Standard bearing formula
    float x = sinf(dlon) * cosf(lat2);
    float y = cosf(lat1) * sinf(lat2) - sinf(lat1) * cosf(lat2) * cosf(dlon);

    float bearing = atan2f(x, y) * RAD_TO_DEG;
    return bearing;  // Already in [-180, 180]
}

float normalize_angle(float angle)
{
    while (angle > 180.0f)  angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

float proportional_navigation(GPSData current, GPSData target, PNState* state)
{
    float bearing = compute_bearing(current, target);
    TickType_t now = xTaskGetTickCount();

    if (!state->initialized)
    {
        state->prev_bearing = bearing;
        state->prev_time = now;
        state->initialized = true;
        return 0.0f;  // No control on first call
    }

    // Compute time delta in seconds
    float dt = (float)(now - state->prev_time) / (float)configTICK_RATE_HZ;
    if (dt < 0.001f)
    {
        return 0.0f;  // Avoid division by zero
    }

    // Bearing rate (degrees per second)
    float bearing_delta = normalize_angle(bearing - state->prev_bearing);
    float bearing_rate = bearing_delta / dt;

    // Update state
    state->prev_bearing = bearing;
    state->prev_time = now;

    // Proportional navigation: turn rate = N * bearing_rate
    return PN_GAIN * bearing_rate;
}

float compute_steering(GPSData current, GPSData target, Orient current_heading, PNState* state)
{
    // Get bearing to target
    float desired_heading = compute_bearing(current, target);

    // Heading error: how much we need to turn
    float heading_error = normalize_angle(desired_heading - current_heading);

    // Proportional navigation correction
    float pn_correction = proportional_navigation(current, target, state);

    // Combine: direct heading error + PN anticipation
    // The heading_error term handles "point at target"
    // The pn_correction term anticipates drift and compensates
    return heading_error + pn_correction;
}

MotorCommand steering_to_motor(float steering)
{
    MotorCommand cmd;

    // Apply deadband - don't actuate for small errors
    if (fabsf(steering) < MOTOR_DEADBAND)
    {
        cmd.direction = 0;
        cmd.pwm = 0;
        return cmd;
    }

    // Direction
    cmd.direction = (steering > 0) ? 1 : -1;

    // Scale steering to PWM (larger error = faster turn)
    float pwm_float = fabsf(steering) * STEERING_GAIN * MOTOR_MAX_PWM / 180.0f;

    // Clamp to valid PWM range
    if (pwm_float > MOTOR_MAX_PWM) pwm_float = MOTOR_MAX_PWM;

    cmd.pwm = (uint8_t)pwm_float;
    return cmd;
}

void ParachuteTask(void* params)
{
    PNState pn_state = { 0.0f, 0, false };

    // Target landing zone (set this to actual coordinates)
    GPSData target = { 0.0f, 0.0f };  // TODO: Set target coordinates

    for (;;)
    {
        // TODO: Receive current GPS and heading from sensor queues
        // GPSData current;
        // Orient heading;
        // xQueueReceive(gps_queue, &current, portMAX_DELAY);
        // xQueueReceive(heading_queue, &heading, portMAX_DELAY);

        // float steering = compute_steering(current, target, heading, &pn_state);
        // MotorCommand cmd = steering_to_motor(steering);
        //
        // Apply to motor:
        //   cmd.direction: -1=left, 0=stop, +1=right
        //   cmd.pwm: 0-255 speed

        vTaskDelay(100 MS);  // 10 Hz control loop
    }
}
