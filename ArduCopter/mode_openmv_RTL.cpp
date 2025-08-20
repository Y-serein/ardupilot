#include "Copter.h"
#include "Parameters.h"
#include <AP_OpenMV/AP_OpenMV.h>
#include "RC_Channel.h"

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
    gcs().send_text(MAV_SEVERITY_INFO, "OPENMV_RTL START");
    temp_x = 2;
    temp_y = 2;
    // start in velaccel control mode
    generate_path();
    pos_control_start();

    rc().init();

    return true;
}

void ModeOpenmvRTL::generate_path()
{
    // float radius_cm = g2.openmv_rtl_cm;

    wp_nav->get_wp_stopping_point(path_ys[0]);

}

// initialise guided mode's position controller
void ModeOpenmvRTL::pos_control_start()
{
    // initialise position controller
    wp_nav->wp_and_spline_init();

    // initialise wpnav to stopping point
    wp_nav->set_wp_destination(path_ys[0], false);

    // 保持当前的偏航角或PID参数
    auto_yaw.set_mode_to_default(false);

    gcs().send_text(MAV_SEVERITY_INFO, "pos_control_start: x=%.2f y=%.2f z=%.2f\r\n", 
                    path_ys[0].x, path_ys[0].y, path_ys[0].z);
}

void ModeOpenmvRTL::run()
{
    if ((temp_x == 0) && (temp_y == 0))
    {
        gcs().send_text(MAV_SEVERITY_INFO, "Draw star finished, now go into loiter mode");
        copter.set_mode(Mode::Number::LOITER, ModeReason::MISSION_END);  // 切换到loiter模式,MISSION_END为切换原因：任务结束自动切换
    }
    else if (wp_nav->reached_wp_destination())                          // 到达某个端点    //wap_nav 航点导航点
    {
        hal.scheduler->delay(1000);
        rc().read_input();
        // gcs().send_text(MAV_SEVERITY_INFO, "temp_x: x=%d\r\n", (int)rc().channel(7)->get_radio_in());
        // gcs().send_text(MAV_SEVERITY_INFO, "temp_y: x=%d\r\n", (int)rc().channel(5)->get_radio_in());
        if ((int)rc().channel(7)->get_radio_in() >= 1750)
        {
            temp_x -= 1.0;
        }
        else if ((int)rc().channel(7)->get_radio_in() <= 1250)
        {
            temp_x += 1.0;
        }
        if ((int)rc().channel(5)->get_radio_in() >= 1750)
        {

            temp_y -= 1.0;
        }
        else if ((int)rc().channel(5)->get_radio_in() <= 1250)
        {
            temp_y += 1.0;
        }
        gcs().send_text(MAV_SEVERITY_INFO, "temp_x=%d temp_y=%d\r\n", temp_x, temp_y);
            
        // path_ys[4] = path_ys[0] + Vector3f(sinf(radians(18.0f)), -cosf(radians(18.0f)), 0) * radius_cm;
        path_ys[1] = path_ys[0] + Vector3f(double(temp_x), double(temp_y), 0);

        gcs().send_text(MAV_SEVERITY_INFO, "pos_control_state: x=%f y=%f z=%f\r\n", path_ys[0].x, path_ys[0].y, path_ys[0].z);

        wp_nav->set_wp_destination(path_ys[1], false);  // 将下一个航点位置设置为导航控制模块的目标位置
    }

    pos_control_run();
    
    path_ys[0] = path_ys[1];


}

// return guided mode timeout in milliseconds. Only used for velocity, acceleration, angle control, and angular rates
uint32_t ModeOpenmvRTL::get_timeout_ms() const
{
    return MAX(copter.g2.guided_timeout, 0.1) * 1000;
}

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