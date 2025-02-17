#include "mp/util/constants.hpp"
#include "mp/vehicles/copter/copter.hpp"
#include "util/logger.hpp"

namespace mp {

vector3f copter::get_linear_acceleration(
    const vector3f& linear_velocity,
    const quaternionf& rotation_q
) const noexcept
{
    const vector3f thrust_force = get_thrust() * rotation_q.rotate_vec(UP);
    const vector3f drag_force = -m_params.lin_drag_c * linear_velocity;

    vector3f acc = GV + (thrust_force + drag_force) / m_params.mass;
    
    // If grounded remove the downwards acceleration component
    if (m_grounded && acc.dot(DOWN) > 0)
        acc *= (!DOWN).cast<float>();
    return acc;
}

vector3f copter::get_angular_acceleration(
    const vector3f& linear_velocity,
    const vector3f& angular_velocity,
    const quaternionf& rotation_q
) const noexcept
{
    if (m_grounded)
        return vector3f(0);

    const matrix3f& I = m_params.moment_of_inertia;
    const vector3f I_w = I.matmul(angular_velocity);

    return (get_torque() - angular_velocity.cross(I_w)).matdivl(I);
}

vehicle::jacobian_s copter::get_jacobian(
    const vector3f& linear_velocity,
    const vector3f& angular_velocity,
    const vector4f& rotation_q
) const noexcept
{
    const float cd = m_params.lin_drag_c;
    const float m = m_params.mass;
    const float T = get_thrust();

    const auto& I = m_params.moment_of_inertia;
    const float Ix = I(0, 0), Iy = I(1, 1), Iz = I(2, 2);
    
    const float qw = rotation_q(0), qx = rotation_q(1), qy = rotation_q(2), qz = rotation_q(3);
    const float wx = angular_velocity(0), wy = angular_velocity(1), wz = angular_velocity(2);

    vector3f da_dv_vec (-cd/m);
    if (m_grounded)
        da_dv_vec *= (!DOWN).cast<float>();

    // Simplified model of the inertia matrix is used (only diagonal elements)
    return jacobian_s {
        .da_dv = da_dv_vec.as_diagonal(),
        .da_dq = (2 * T / m) * matrixf<3, 4> {
            {qy, qz, qw, qx},
            {-qx, -qw, qz, qy},
            {qw, -qx, -qy, qz}
        },
        .ddw_dv = matrixf<3>(0),
        .ddw_dw = matrixf<3> {
            {0, (Iy-Iz)*wz/Ix, (Iy-Iz)*wy/Ix},
            {(Iz-Ix)*wz/Iy, 0, (Iz-Ix)*wx/Iy},
            {(Ix-Iy)*wy/Iz, (Ix-Iy)*wx/Iz, 0}
        },
        .ddw_dq = matrixf<3, 4>(0)
    };
}

void copter::update_grounded(const state_s& state) noexcept
{
    static constexpr float TAKEOFF_VELOCITY_THRESHOLD = 0.05f;
    static constexpr float STATIONARY_SPEED_SQ_THRESHOLD = 0.01f;
    static constexpr float STATIONARY_ACC_MINIMUM_DIFF = 1.f;

    // Check to see if we are most likely on ground
    if (m_grounded) {
        if (state.velocity.dot(UP) > TAKEOFF_VELOCITY_THRESHOLD) {
            m_grounded = false;
            log_info("Copter takeoff!");
        }
    } else {
        bool stationary = state.velocity.norm_sq() < STATIONARY_SPEED_SQ_THRESHOLD;

        // This expected acceleration is calculated assuming the grounded is false
        // since we're in this branch of the if expression
        const vector3f acceleration_expected = get_linear_acceleration(state.velocity, state.rotationq);
        const vector3f acceleration_diff = state.acceleration - acceleration_expected;

        // If there is less acceleration downwards than expected and we're stationary,
        // we're probably grounded
        if (acceleration_diff.dot(DOWN) < STATIONARY_ACC_MINIMUM_DIFF && stationary) {
            m_grounded = true;
            log_info("Copter landing!");
        }
    }
}

void copter::update(const state_s& state, float dt) noexcept
{
    update_grounded(state);
    
    m_controller.update(state, dt);
}

}