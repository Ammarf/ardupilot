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
    _desired_yaw_cd = wrap_180_cd(atan2f(scaled_steering,scaled_throttle) * DEGX100);

    // reset heading when the vehicle is stopped
    if (throttle == 0.0f) {
        g2.motors.set_throttle(0.0f);
    }

    // run steering and throttle controllers
    calc_steering_to_heading(_desired_yaw_cd, throttle < 0);

    g2.motors.set_throttle(throttle * 100.0f);
}
