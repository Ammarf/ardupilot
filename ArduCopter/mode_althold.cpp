#include "Copter.h"

bool right_enable = false;
bool left_enable = false;
bool up_enable = false;

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

    float right_lidar_input = copter.rangefinder.distance_cm_orient(ROTATION_YAW_90) * ahrs.cos_roll();
    float left_lidar_input = copter.rangefinder.distance_cm_orient(ROTATION_YAW_270) * ahrs.cos_roll();
    float upward_lidar_input = copter.rangefinder.distance_cm_orient(ROTATION_PITCH_90) * MAX(0.707f, ahrs.get_rotation_body_to_ned().c.z);

    gcs().send_text(MAV_SEVERITY_INFO, "right: %f, left: %f, up: %f", right_lidar_input, left_lidar_input, upward_lidar_input);

    return true;
}

// althold_run - runs the althold controller
// should be called at 100hz or more
void Copter::ModeAltHold::run()
{
    AltHoldModeState althold_state;
    float takeoff_climb_rate = 0.0f;

    RC_Channel *rc11 = RC_Channels::rc_channel(CH_11);
    RC_Channel *rc10 = RC_Channels::rc_channel(CH_10);
    RC_Channel *rc14 = RC_Channels::rc_channel(CH_14);

    copter.rangefinder.update();

    float right_lidar_input = copter.rangefinder.distance_cm_orient(ROTATION_YAW_90) * ahrs.cos_roll();
    float left_lidar_input = copter.rangefinder.distance_cm_orient(ROTATION_YAW_270) * ahrs.cos_roll();
    float upward_lidar_input = copter.rangefinder.distance_cm_orient(ROTATION_PITCH_90) * MAX(0.707f, ahrs.get_rotation_body_to_ned().c.z);

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

        uint16_t radio11_in = rc11->get_radio_in(); // ENABLE SIDE LIDARS
        uint16_t radio10_in = rc10->get_radio_in(); // SWITCH BETWEEN RIGHT AND LEFT SIDE LIDAR
        uint16_t radio14_in = rc14->get_radio_in(); // ENABLE UPWARD LIDAR


        if (radio11_in > 1700 && radio10_in < 1700 && right_lidar_input > 10 && right_lidar_input < 600) {
            float right_setpoint = g2.side_setpoint;
            float right_lidar_error = right_lidar_input - right_setpoint;

            float right_lidar_error_max = 1.00f;
            float right_lidar_error_factor = right_lidar_error / right_lidar_error_max;

            g2.right_hold_pid.set_input_filter_all(right_lidar_error);
            float right_control_p = g2.right_hold_pid.get_p();
            float right_control_i = g2.right_hold_pid.get_i();
            float right_control_d = g2.right_hold_pid.get_d();
            float right_control_ff = g2.right_hold_pid.get_ff(right_lidar_error_factor);

            float right_pid_output = right_control_p + right_control_i + right_control_d + right_control_ff;

            if (right_enable == false) {
                gcs().send_text(MAV_SEVERITY_INFO, "right: %f, set: %f, pid: %f", right_lidar_input, right_setpoint, right_pid_output);
                right_enable = true;
            }
            // call attitude controller
            attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(right_pid_output, target_pitch, target_yaw_rate);
        } else {
            right_enable = false;

            // call attitude controller
            attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);
        }

        if (radio11_in > 1700 && radio10_in > 1700 && left_lidar_input > 10 && left_lidar_input < 600) {
        	float left_setpoint = g2.side_setpoint;
        	float left_lidar_error = left_lidar_input - left_setpoint;

        	float left_lidar_error_max = 1.00f;
        	float left_lidar_error_factor = left_lidar_error / left_lidar_error_max;

        	g2.left_hold_pid.set_input_filter_all(left_lidar_error);
        	float left_control_p = g2.left_hold_pid.get_p();
        	float left_control_i = g2.left_hold_pid.get_i();
        	float left_control_d = g2.left_hold_pid.get_d();
        	float left_control_ff = g2.left_hold_pid.get_ff(left_lidar_error_factor);

        	float left_pid_output = left_control_p + left_control_i + left_control_d + left_control_ff;

        	if (left_enable == false) {
        		gcs().send_text(MAV_SEVERITY_INFO, "left: %f, set: %f, pid: %f", left_lidar_input, left_setpoint, left_pid_output);
        		left_enable = true;
        	}
        	// call attitude controller
        	attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(-left_pid_output, target_pitch, target_yaw_rate);
        } else {
        	left_enable = false;

        	// call attitude controller
        	attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);
        }

        if (radio14_in > 1700 && upward_lidar_input > 10 && upward_lidar_input < 800) {
            float upward_setpoint = g2.upward_setpoint;
            float upward_lidar_error = upward_lidar_input - upward_setpoint;

            float upward_lidar_error_max = 1.00f;
            float upward_lidar_error_factor = upward_lidar_error / upward_lidar_error_max;

            g2.upward_hold_pid.set_input_filter_all(upward_lidar_error);
            float upward_control_p = g2.upward_hold_pid.get_p();
            float upward_control_i = g2.upward_hold_pid.get_i();
            float upward_control_d = g2.upward_hold_pid.get_d();
            float upward_control_ff = g2.upward_hold_pid.get_ff(upward_lidar_error_factor);

            float upward_pid_output = -(upward_control_p + upward_control_i + upward_control_d + upward_control_ff);

            target_climb_rate = get_pilot_desired_climb_rate(485 - upward_pid_output);

            if (up_enable == false) {
                gcs().send_text(MAV_SEVERITY_INFO, "up: %f, set: %f, pid: %f", upward_lidar_input, upward_setpoint, upward_pid_output);
                up_enable = true;
            }

            // adjust climb rate using rangefinder
            target_climb_rate = get_surface_tracking_climb_rate(target_climb_rate, pos_control->get_alt_target(), G_Dt);

            // get avoidance adjusted climb rate
            target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

            // call position controller
            pos_control->set_alt_target_from_climb_rate_ff(target_climb_rate, G_Dt, false);
            pos_control->update_z_controller();
            break;
        } else {
            up_enable = false;

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
