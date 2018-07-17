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

    // cruise mode code
/*
    desired_throttle = constrain_float(desired_throttle, -100.0f, 100.0f);
    desired_steering = constrain_float(desired_steering, -4500.0f, 4500.0f);

    // scale steering and throttle inputs to -1 to +1 range
    float scaled_throttle = desired_throttle / 100.0f;
    float scaled_steering = desired_steering / 4500.0f;

    float magnitude_max = 1.0;
    float theta_max = 1.0;
    const float magnitude = safe_sqrt((scaled_throttle*scaled_throttle)+(scaled_steering*scaled_steering));
    float theta =  0;
    float theta_cd = 0;
    //float theta = atan2f(scaled_throttle,scaled_steering);

    if (fabsf(magnitude) > magnitude_max) {
        magnitude_max = magnitude;
    }

    float magnitude_final = magnitude / magnitude_max;


    // 1st quadrant
    if ((scaled_throttle > 0) && (scaled_steering > 0)) {
        theta = wrap_180_cd(atan2f(scaled_steering,scaled_throttle) * DEGX100);
        theta_cd = theta;
    }
    // 2nd quadrant
    if ((scaled_throttle < 0) && (scaled_steering > 0)) {
        theta = wrap_180_cd(atan2f(scaled_steering,scaled_throttle) * DEGX100);
        theta_cd = (180.0 * DEGX100 - theta);
    }
    // 3rd quadrant
    if ((scaled_throttle < 0) && (scaled_steering < 0)) {
        theta = wrap_180_cd(atan2f(scaled_steering,scaled_throttle) * DEGX100);
        theta_cd = (270.0 * DEGX100 - theta);
    }
    // 4th quadrant
    if ((scaled_throttle > 0) && (scaled_steering < 0)) {
        theta = wrap_180_cd(atan2f(scaled_steering,scaled_throttle) * DEGX100);
        theta_cd = (360.0 * DEGX100 - theta);
    }


    theta_cd = (atan2f(scaled_steering,scaled_throttle) * DEGX100);

    if (fabsf(theta_cd) > theta_max) {
        theta_max = theta_cd;
    }

    float theta_final = theta_cd;
*/
    hal.console->printf("throttle is %lf \n", desired_throttle);
    //hal.console->printf("steering is %lf \n", scaled_steering);
    //hal.console->printf("theta final is %lf \n", theta_final);
    //hal.console->printf("yaw is %d \n", ahrs.yaw_sensor);
    //hal.console->printf("steering output is %lf \n", (theta_final));

    // copy RC scaled inputs to outputs
    //g2.motors.set_throttle(magnitude_final * 100.0f);
    //calc_steering_to_heading((0 - theta_final) * 4500.0f, _desired_speed < 0);
    //g2.motors.set_steering((0 - theta_final) * 4500.0f, false);
}
