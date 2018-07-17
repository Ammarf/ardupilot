#include "mode.h"
#include "Rover.h"

void ModeManual::_exit()
{
    // clear lateral when exiting manual mode
    g2.motors.set_lateral(0);
}

void ModeManual::update()
{
    float desired_steering, desired_throttle, desired_lateral;
    get_pilot_desired_steering_and_throttle(desired_steering, desired_throttle);
    get_pilot_desired_lateral(desired_lateral);

    // if vehicle is balance bot, calculate actual throttle required for balancing
    if (rover.is_balancebot()) {
        rover.balancebot_pitch_control(desired_throttle, rover.arming.is_armed());
    }

    desired_throttle = constrain_float(desired_throttle, -100.0f, 100.0f);
    desired_steering = constrain_float(desired_steering, -4500.0f, 4500.0f);

    // scale steering and throttle down to a -1 to +1 range
    float scaled_throttle = desired_throttle / 100.0f;
    float scaled_steering = desired_steering / 4500.0f;

    float magnitude_max = 1.0f;
    float theta_max = 1.0f;

    float magnitude = safe_sqrt(sq(scaled_throttle) + sq(scaled_steering));
    if (fabsf(magnitude) > magnitude_max) {
        magnitude_max = magnitude;
    }

    float speed = magnitude / magnitude_max;

    if (is_negative(scaled_throttle)) {
        scaled_steering *= -1.0f;
    }

    float theta = wrap_360_cd(atan2f(scaled_steering,scaled_throttle) * DEGX100);
    //float desired_heading = ahrs.yaw_sensor - theta;

    hal.console->printf("theta is %lf \n", theta);
    hal.console->printf("yaw is %d \n", ahrs.yaw_sensor);


    // copy RC scaled inputs to outputs
    calc_steering_to_heading(theta, _desired_speed < 0);
    calc_throttle(speed, false, true);
    //g2.motors.set_throttle(magnitude * 100.0f);
    //g2.motors.set_steering(radians(desired_heading * 0.01f) * 4500.0f, false);

}
