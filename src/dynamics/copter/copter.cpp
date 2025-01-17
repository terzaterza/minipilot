#include "copter.hpp"

namespace mp {

emblib::vector3f copter::acc(
    const emblib::vector3f& vel,
    const emblib::quaternionf& rotq
) const noexcept
{
    const float m = m_params.mass;
    return GRAVITY + rotq.rotate_vec((thrust() / m) * UP) - m_params.lin_drag_c * vel / m;
}

emblib::vector3f copter::ang_acc(
    const emblib::vector3f& vel,
    const emblib::quaternionf& rotq,
    const emblib::vector3f& ang_vel
) const noexcept
{
    const emblib::matrixf<3>& I = m_params.inertia_matrix;
    const emblib::vector3f I_w = I.matmul(ang_vel);
    return (torque() - ang_vel.cross(I_w)).matdivl(I);
}

model::jacobian_s copter::jacobian(
    const emblib::vector3f& vel,
    const emblib::vectorf<4>& rotqv,
    const emblib::vector3f& ang_vel
) const noexcept
{
    const float m = m_params.mass;
    const float T = thrust();

    const auto& I = m_params.inertia_matrix;
    const float Ix = I(0, 0), Iy = I(1, 1), Iz = I(2, 2);
    
    const float qw = rotqv(0), qx = rotqv(1), qy = rotqv(2), qz = rotqv(3);
    const float wx = ang_vel(0), wy = ang_vel(1), wz = ang_vel(2);

    // Simplified model of the inertia matrix is used (only diagonal elements)
    return jacobian_s {
        .acc_vel = static_cast<emblib::vector3f>(-m_params.lin_drag_c / m).as_diagonal(),
        .acc_rotq = (2 * T / m) * emblib::matrixf<3, 4> {
            {qy, qz, qw, qx},
            {-qx, -qw, qz, qy},
            {qw, -qx, -qy, qz}
        },
        .ang_acc_vel = emblib::matrixf<3>(0),
        .ang_acc_rotq = emblib::matrixf<3, 4>(0),
        .ang_acc_ang_vel = emblib::matrixf<3> {
            {0, (Iy-Iz)*wz/Ix, (Iy-Iz)*wy/Ix},
            {(Iz-Ix)*wz/Iy, 0, (Iz-Ix)*wx/Iy},
            {(Ix-Iy)*wy/Iz, (Ix-Iy)*wx/Iz, 0}
        }
    };
}

}