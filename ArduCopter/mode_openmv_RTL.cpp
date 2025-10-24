#include "Copter.h"
#include "Parameters.h"
#include <AP_HAL/Semaphores.h>

#if MODE_OPENMVRTL_ENABLED == ENABLED

AP_OpenMV openmv{};
/*
 * Init and run calls for guided flight mode
 */

// static Vector3p guided_pos_target_cm;       // position target (used by posvel controller only)
// bool guided_pos_terrain_alt_ys;                // true if guided_pos_target_cm.z is an alt above terrain
// static Vector3f guided_vel_target_cms;      // velocity target (used by pos_vel_accel controller and vel_accel controller)
// static Vector3f guided_accel_target_cmss;   // acceleration target (used by pos_vel_accel controller vel_accel controller and accel controller)
// static uint32_t update_time_ms;             // system time of last target update to pos_vel_accel, vel_accel or accel controller

// struct {
//     uint32_t update_time_ms;
//     Quaternion attitude_quat;
//     Vector3f ang_vel;
//     float yaw_rate_cds;
//     float climb_rate_cms;   // climb rate in cms.  Used if use_thrust is false
//     float thrust;           // thrust from -1 to 1.  Used if use_thrust is true
//     bool use_yaw_rate;
//     bool use_thrust;
// } static guided_angle_state;

// init - initialise guided controller

bool ModeOpenmvRTL::init(bool ignore_checks)
{
    path_num_ys = 0;

    // start in velaccel control mode
    generate_path();
    pos_control_start();

    gcs().send_text(MAV_SEVERITY_INFO, "[Serein_Y] OPENMV_RTL START\r\n");
    return true;
}

void ModeOpenmvRTL::generate_path()
{
    float radius_cm = g2.ze_star_alt_cm;

    wp_nav->get_wp_stopping_point(path_ys[0]);

    path_ys[1] = path_ys[0] + Vector3f(50.0f, 0, 50.0f) * radius_cm;
    path_ys[2] = path_ys[0] + Vector3f(0, 0, -path_ys[0].z);
    gcs().send_text(MAV_SEVERITY_INFO, "[Serein_Y] path_1: x=%.6f y=%.6f z=%.6f\r\n", 
        path_ys[2].x, path_ys[2].y, path_ys[2].z);
    gcs().send_text(MAV_SEVERITY_INFO, "[Serein_Y] path_1: x=%.6f y=%.6f z=%.6f\r\n", 
        path_ys[0].z, path_ys[1].z, path_ys[2].z);


}

/* ---------------------Serein_Y Start-----------------------------------------*/

void ModeOpenmvRTL::pos_control_start()
{
    // initialise position controller
    wp_nav->wp_and_spline_init();

    // initialise wpnav to stopping point
    wp_nav->set_wp_destination(path_ys[0], false);

    auto_yaw.set_mode_to_default(false);

    gcs().send_text(MAV_SEVERITY_CRITICAL, "[Serein_Y] pos_control_start!\r\n");
}

bool ModeOpenmvRTL::check_reaching_rtl_altitude_cm()
{
    Vector3f current_loc;
    //int32_t rtl_height = copter.g.rtl_altitude;
    int32_t rtl_height = 300;
    wp_nav->get_wp_stopping_point(current_loc);

    //gcs().send_text(MAV_SEVERITY_INFO, "[Serein_Y] rtl_altitude_cm: x=%.2f y=%.2f z=%.2f\r\n", 
    //        current_loc.x, current_loc.y, current_loc.z);

    return (current_loc.z <= rtl_height);
    //return ((current_loc.z <= rtl_height) && return_to_home_start_ys())?true:false;
    
}

bool ModeOpenmvRTL::return_to_home_start_ys()
{
    Location loc;
    Location home_loc;

    AP_AHRS &_ahrs = AP::ahrs();
    //WITH_SEMAPHORE(_ahrs.get_semaphore());
    
    if (_ahrs.get_location(loc) && _ahrs.home_is_set())
    {
        home_loc = _ahrs.get_home();
        home_loc.alt += 50;
        gcs().send_text(MAV_SEVERITY_INFO, "[Serein_Y] home_loc_cm: x=%d y=%d z=%d\r\n", home_loc.alt, home_loc.lat, home_loc.lng);
        // copter.mode_guided.set_destination(home_loc);

        wp_nav->set_wp_destination(path_ys[2], false);
        return wp_nav->reached_wp_destination();
    }

    gcs().send_text(MAV_SEVERITY_CRITICAL, "[Serein_Y] Home not set! Entering RTL, x=%d y=%d z=%d\r\n", home_loc.alt, home_loc.lat, home_loc.lng);
    copter.set_mode(Mode::Number::RTL, ModeReason::MISSION_END);
    return false;
}

void ModeOpenmvRTL::run()
{   
    if (wp_nav->reached_wp_destination())
    {
        gcs().send_text(MAV_SEVERITY_INFO, "[Serein_Y] now go into Next pos: %d\r\n", path_num_ys);
            
        if (path_num_ys < 2)
        {
            path_num_ys++;
            wp_nav->set_wp_destination(path_ys[path_num_ys], false);
        }
        else
        {
            gcs().send_text(MAV_SEVERITY_INFO, "[Serein_Y] ERROR\r\n");
            copter.set_mode(Mode::Number::LOITER, ModeReason::MISSION_END);
        }
    }
    if ((path_num_ys == 2) && check_reaching_rtl_altitude_cm())
    {
        gcs().send_text(MAV_SEVERITY_INFO, "[Serein_Y] Starting landing procedure\r\n");
        copter.set_mode(Mode::Number::LAND, ModeReason::MISSION_END);
    }
    
    pos_control_run();
}

/* ---------------------Serein_Y end-----------------------------------------*/

void ModeOpenmvRTL::pos_control_run()
{
    // process pilot's yaw input
    float target_yaw_rate = 0;
    if (!copter.failsafe.radio && use_pilot_yaw()) {
        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->norm_input_dz());
        if (!is_zero(target_yaw_rate)) {
            auto_yaw.set_mode(AutoYaw::Mode::HOLD);
        }
    }
    // if not armed set throttle to zero and exit immediately
    if (is_disarmed_or_landed()) {
        // do not spool down tradheli when on the ground with motor interlock enabled
        make_safe_ground_handling(copter.is_tradheli() && motors->get_interlock());
        return;
    }
    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
    // run waypoint controller
    copter.failsafe_terrain_set_status(wp_nav->update_wpnav());
    // call z-axis position controller (wpnav should have already updated it's alt target)
    pos_control->update_z_controller();
    // call attitude controller
    attitude_control->input_thrust_vector_heading(pos_control->get_thrust_vector(), auto_yaw.get_heading());
}

#endif