#include "mp/util/constants.hpp"
#include "mp/vehicles/copter/copter.hpp"

namespace mp {

vector3f copter::get_linear_acceleration(
    const vector3f& linear_velocity,
    const quaternionf& rotation_q
) const noexcept
{
    const vector3f thrust_force = get_thrust() * rotation_q.rotate_vec(UP);
    const vector3f drag_force = -m_params.lin_drag_c * linear_velocity;

    return GV + (thrust_force + drag_force) / m_params.mass;
}

vector3f copter::get_angular_acceleration(
    const vector3f& linear_velocity,
    const vector3f& angular_velocity,
    const quaternionf& rotation_q
) const noexcept
{
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
    const float m = m_params.mass;
    const float T = get_thrust();

    const auto& I = m_params.moment_of_inertia;
    const float Ix = I(0, 0), Iy = I(1, 1), Iz = I(2, 2);
    
    const float qw = rotation_q(0), qx = rotation_q(1), qy = rotation_q(2), qz = rotation_q(3);
    const float wx = angular_velocity(0), wy = angular_velocity(1), wz = angular_velocity(2);

    // Simplified model of the inertia matrix is used (only diagonal elements)
    return jacobian_s {
        .da_dv = matrix3f::diagonal(-m_params.lin_drag_c),
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

void copter::update(const state_s& state, float dt) noexcept
{
    
}

}