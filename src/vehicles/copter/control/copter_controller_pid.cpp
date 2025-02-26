#include "copter_controller_pid.hpp"
#include "mp/util/constants.hpp"
#include <cmath>

namespace mp {

copter_controller_pid::copter_controller_pid(const copter_params_s& copter_params) noexcept :
    m_copter_params(copter_params),
    m_angular_velocity_pid(1, 0.2, 0),
    m_linear_acceleration_pid(1, 2, 0),
    m_control_mode(control_mode_e::ANGULAR)
{}

bool copter_controller_pid::set_target_w(const vector3f& target_w, float target_thrust) noexcept
{
    // TODO: Add bounds checking and return false if out of bounds
    m_control_mode = control_mode_e::ANGULAR;
    m_target_w = target_w;
    m_output_thrust = target_thrust;
    return true;
}

bool copter_controller_pid::set_target_v(const vector3f& target_v, float target_dir) noexcept
{
    // TODO: Add bounds checking
    // TODO: Reset linear pid to avoid previous integral accumulation glitches
    m_control_mode = control_mode_e::LINEAR;
    m_target_v = target_v;
    m_target_dir = target_dir;
    return true;
}

void copter_controller_pid::update(const state_s& state, float dt) noexcept
{
    if (m_control_mode == control_mode_e::LINEAR) {
        const vector3f& v = state.velocity;

        // Scaling constant should be determined based on maximum velocity and maximum acceleration
        m_linear_acceleration_pid.update(m_target_v - v, dt);
        const vector3f target_a = m_linear_acceleration_pid.get_output();
        // Target thrust vector in the global frame - derived from the linear acceleration equation of the copter
        const vector3f target_thrust_g = m_copter_params.mass * (target_a - GV) + m_copter_params.lin_drag_c * v;
        // Target thrust vector in the body (local) frame
        const vector3f target_thrust_l = state.rotationq.conjugate().rotate_vec(target_thrust_g);
        // Direction of the angular velocity vector required to rotate local UP to local target thrust
        const vector3f target_w_dir = UP.cross(target_thrust_l.normalized());

        // Since the cross product is between to normalized vectors, its max magnitude is
        // 1 when the angle is PI/2, so this constant is the maximum magnitude of target w
        m_target_w = target_w_dir * 5.f;

        // TODO: Add yaw rotation based on m_target_dir

        m_output_thrust = target_thrust_g.norm();
    }

    const vector3f& w = state.angular_velocity;

    // target_dw = (target_w - w) * PID(s)
    m_angular_velocity_pid.update(m_target_w - w, dt);
    vector3f target_dw = m_angular_velocity_pid.get_output();

    const matrix3f& I = m_copter_params.moment_of_inertia;
    m_output_torque = I.matmul(target_dw) + w.cross(static_cast<vector3f>(I.matmul(w)));
}

}