#include "mode.h"
#include "Rover.h"

void ModeManual::_exit()
{
    // clear lateral when exiting manual mode
    g2.motors.set_lateral(0);
}

bool ModeManual::_enter()
{
    //initialize stuff
    _desired_yaw_cd = ahrs.yaw_sensor;
    _yaw_error_cd = 0.0f;

    return true;
}

void ModeManual::update()
{
    // get pilot input
    float desired_steering, desired_throttle;
    get_pilot_desired_steering_and_throttle(desired_steering, desired_throttle);

    //hal.console->printf("throttle is %lf \n", desired_throttle);

    // constrain steering and throttle inputs
    desired_throttle = constrain_float(desired_throttle, -100.0f, 100.0f);
    desired_steering = constrain_float(desired_steering, -4500.0f, 4500.0f);

    // scale steering and throttle down to a -1 to +1 range
    float scaled_throttle = desired_throttle / 100.0f;
    float scaled_steering = desired_steering / 4500.0f;

    // calculate magnitude of input stick vector
    float magnitude_max = 1.0f;
    float magnitude = safe_sqrt(sq(scaled_throttle) + sq(scaled_steering));
    if (fabsf(magnitude) > magnitude_max) {
        magnitude_max = magnitude;
    }
    float throttle = magnitude / magnitude_max;

    //hal.console->printf("throttle is %lf \n", throttle);


    // reverse steering when throttle is negative to correct heading angle
    if (is_negative(scaled_throttle)) {
        scaled_steering *= -1.0f;
    }

    // calculate angle of input stick vector
    _desired_yaw_cd = wrap_360_cd(atan2f(scaled_steering,scaled_throttle) * DEGX100);

    //_yaw_error_cd = ahrs.sin_yaw();
    float final_heading1 = _desired_yaw_cd - ahrs.yaw_sensor;

    if (is_negative(scaled_steering)) {
        final_heading1 = 36000.0f - (_desired_yaw_cd - ahrs.yaw_sensor);
    }

    float final_heading2 = radians(final_heading1 * 0.01f);

    float final_heading_max = 1.0f;

    if (fabsf(final_heading2) > final_heading_max) {
        final_heading_max = final_heading2;
    }

    float final_heading3 = final_heading2 / final_heading_max;

    if (scaled_steering < 0.0f) {
        final_heading3 = final_heading3 * -1.0f;
    }

    hal.console->printf("yaw_error is %lf \n", final_heading3);

    g2.motors.set_steering(final_heading3 * 4500.0f, false);
    g2.motors.set_throttle(throttle * 100.0f);
}
