#include "Copter.h"


/*
 * Init and run calls for althold, flight mode
 */

// althold_init - initialise althold controller
bool Copter::ModeAltHold::init(bool ignore_checks)
{
    // initialise position and desired velocity
    if (!pos_control->is_active_z()) {
        pos_control->set_alt_target_to_current_alt();
        pos_control->set_desired_velocity_z(inertial_nav.get_velocity_z());
    }

    return true;
}

// althold_run - runs the althold controller
// should be called at 100hz or more
void Copter::ModeAltHold::run()
{
    AltHoldModeState althold_state;
    float takeoff_climb_rate = 0.0f;

    RC_Channel *rc12 = RC_Channels::rc_channel(CH_12);
    RC_Channel *rc14 = RC_Channels::rc_channel(CH_14);

    copter.rangefinder.update();

    float lidar_input = copter.rangefinder.distance_cm_orient(ROTATION_YAW_90) * ahrs.cos_roll();;
    float upward_lidar_input = copter.rangefinder.distance_cm_orient(ROTATION_PITCH_90) * MAX(0.707f, ahrs.get_rotation_body_to_ned().c.z);

    //hal.console->printf("lidar_input is %lf \n", lidar_input);
    //hal.console->printf("upward_lidar_input is %lf \n", upward_lidar_input);

    // initialize vertical speeds and acceleration
    pos_control->set_speed_z(-get_pilot_speed_dn(), g.pilot_speed_up);
    pos_control->set_accel_z(g.pilot_accel_z);

    // apply SIMPLE mode transform to pilot inputs
    update_simple_mode();

    // get pilot desired lean angles
    float target_roll, target_pitch;
    get_pilot_desired_lean_angles(target_roll, target_pitch, copter.aparm.angle_max, attitude_control->get_althold_lean_angle_max());

    // get pilot's desired yaw rate
    float target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());

    // get pilot desired climb rate
    float target_climb_rate = get_pilot_desired_climb_rate(channel_throttle->get_control_in());
    target_climb_rate = constrain_float(target_climb_rate, -get_pilot_speed_dn(), g.pilot_speed_up);

    // Alt Hold State Machine Determination
    if (!motors->armed() || !motors->get_interlock()) {
        althold_state = AltHold_MotorStopped;
    } else if (takeoff.running() || takeoff.triggered(target_climb_rate)) {
        althold_state = AltHold_Takeoff;
    } else if (!ap.auto_armed || ap.land_complete) {
        althold_state = AltHold_Landed;
    } else {
        althold_state = AltHold_Flying;
    }

    // Alt Hold State Machine
    switch (althold_state) {

    case AltHold_MotorStopped:

        motors->set_desired_spool_state(AP_Motors::DESIRED_SHUT_DOWN);
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);
        attitude_control->reset_rate_controller_I_terms();
        attitude_control->set_yaw_target_to_current_heading();
#if FRAME_CONFIG == HELI_FRAME    
        // force descent rate and call position controller
        pos_control->set_alt_target_from_climb_rate(-abs(g.land_speed), G_Dt, false);
        heli_flags.init_targets_on_arming=true;
        if (ap.land_complete_maybe) {
            pos_control->relax_alt_hold_controllers(0.0f);
        }
#else
        pos_control->relax_alt_hold_controllers(0.0f);   // forces throttle output to go to zero
#endif
        pos_control->update_z_controller();
        break;

    case AltHold_Takeoff:
#if FRAME_CONFIG == HELI_FRAME    
        if (heli_flags.init_targets_on_arming) {
            heli_flags.init_targets_on_arming=false;
        }
#endif
        // set motors to full range
        motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

        // initiate take-off
        if (!takeoff.running()) {
            takeoff.start(constrain_float(g.pilot_takeoff_alt,0.0f,1000.0f));
            // indicate we are taking off
            set_land_complete(false);
            // clear i terms
            set_throttle_takeoff();
        }

        // get take-off adjusted pilot and takeoff climb rates
        takeoff.get_climb_rates(target_climb_rate, takeoff_climb_rate);

        // get avoidance adjusted climb rate
        target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

        // call attitude controller
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);

        // call position controller
        pos_control->set_alt_target_from_climb_rate_ff(target_climb_rate, G_Dt, false);
        pos_control->add_takeoff_climb_rate(takeoff_climb_rate, G_Dt);
        pos_control->update_z_controller();
        break;

    case AltHold_Landed:
        // set motors to spin-when-armed if throttle below deadzone, otherwise full range (but motors will only spin at min throttle)
        if (target_climb_rate < 0.0f) {
            motors->set_desired_spool_state(AP_Motors::DESIRED_SPIN_WHEN_ARMED);
        } else {
            motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);
        }

#if FRAME_CONFIG == HELI_FRAME    
        if (heli_flags.init_targets_on_arming) {
            attitude_control->reset_rate_controller_I_terms();
            attitude_control->set_yaw_target_to_current_heading();
            if (motors->get_interlock()) {
                heli_flags.init_targets_on_arming=false;
            }
        }
#else
        attitude_control->reset_rate_controller_I_terms();
        attitude_control->set_yaw_target_to_current_heading();
#endif
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);
        pos_control->relax_alt_hold_controllers(0.0f);   // forces throttle output to go to zero
        pos_control->update_z_controller();
        break;

    case AltHold_Flying:
        motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

#if AC_AVOID_ENABLED == ENABLED
        // apply avoidance
        copter.avoid.adjust_roll_pitch(target_roll, target_pitch, copter.aparm.angle_max);
#endif

        uint16_t radio12_in = rc12->get_radio_in();
        uint16_t radio14_in = rc14->get_radio_in();

        //hal.console->printf("lidar_input is %lf \n", lidar_input);

        if (radio12_in > 1800 && lidar_input < 600) {

            float setpoint = (radio14_in - 1094) * (600.0f - 200.0f) / (1934 - 1094) + 200.0f;
            float lidar_error = lidar_input - setpoint;

            float lidar_error_max = 1.00f;
            float lidar_error_factor = lidar_error / lidar_error_max;

            g2.hold_pid.set_input_filter_all(lidar_error);
            float control_p = g2.hold_pid.get_p();
            float control_i = g2.hold_pid.get_i();
            float control_d = g2.hold_pid.get_d();
            float control_ff = g2.hold_pid.get_ff(lidar_error_factor);

            //hal.console->printf("lidar_error_factor is %lf \n", lidar_error_factor);
            //hal.console->printf("p is %lf, i is %lf, d is %lf, ff is %lf \n", control_p, control_i, control_d, control_ff);

            float pid_output = control_p + control_i + control_d + control_ff;

/*
            // UOWARD LIDAR PID
            float upward_setpoint = 400.0f;
            float upward_lidar_error = upward_lidar_input - upward_setpoint;

            float upward_lidar_error_max = 1.00f;
            float upward_lidar_error_factor = upward_lidar_error / upward_lidar_error_max;


            g2.upward_hold_pid.set_input_filter_all(upward_lidar_error);
            float upward_control_p = g2.upward_hold_pid.get_p();
            float upward_control_i = g2.upward_hold_pid.get_i();
            float upward_control_d = g2.upward_hold_pid.get_d();
            float upward_control_ff = g2.upward_hold_pid.get_ff(upward_lidar_error_factor);

            float upward_pid_output = upward_control_p + upward_control_i + upward_control_d + upward_control_ff;
            //upward_pid_output = constrain_float(upward_pid_output, -10.0f, 10.0f);

            target_climb_rate = get_pilot_desired_climb_rate(500.0f - upward_pid_output);

            //hal.console->printf("target_climb_rate is %lf \n", upward_pid_output);
            //hal.console->printf("target_climb_rate is %lf \n", target_climb_rate);
            //hal.console->printf("p is %lf, i is %lf, d is %lf, ff is %lf \n", upward_control_p, upward_control_i, upward_control_d, upward_control_ff);
*/
            // call attitude controller
            attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(pid_output, target_pitch, target_yaw_rate);

            // adjust climb rate using rangefinder
            target_climb_rate = get_surface_tracking_climb_rate(target_climb_rate, pos_control->get_alt_target(), G_Dt);

            // get avoidance adjusted climb rate
            target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

            // call position controller
            pos_control->set_alt_target_from_climb_rate_ff(target_climb_rate, G_Dt, false);
            pos_control->update_z_controller();
            break;

        } else {

            // call attitude controller
            attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);

            // adjust climb rate using rangefinder
            target_climb_rate = get_surface_tracking_climb_rate(target_climb_rate, pos_control->get_alt_target(), G_Dt);

            // get avoidance adjusted climb rate
            target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

            // call position controller
            pos_control->set_alt_target_from_climb_rate_ff(target_climb_rate, G_Dt, false);
            pos_control->update_z_controller();
            break;

        }
    }
}
