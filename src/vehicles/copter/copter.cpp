#include "mp/util/constants.hpp"
#include "mp/vehicles/copter/copter.hpp"
#include "util/logger.hpp"

namespace mp {

vector3f copter::get_linear_acceleration(
    const vector3f& v,
    const quaternionf& q
) const noexcept
{
    const vector3f thrust_force = get_thrust() * q.rotate_vec(UP);
    const vector3f drag_force = -m_params.lin_drag_c * v;

    vector3f acc = GV + (thrust_force + drag_force) / m_params.mass;
    
    // If grounded remove the downwards acceleration component
    if (m_grounded && acc.dot(DOWN) > 0)
        acc *= (!DOWN).cast<float>();
    return acc;
}

vector3f copter::get_angular_acceleration(
    const vector3f& v,
    const vector3f& w,
    const quaternionf& q
) const noexcept
{
    if (m_grounded)
        return vector3f(0);

    const matrix3f& I = m_params.moment_of_inertia;
    const vector3f I_w = I.matmul(w);

    return (get_torque() - w.cross(I_w)).matdivl(I);
}

copter::jacobian_s copter::get_jacobian(
    const vector3f& v,
    const vector3f& w,
    const vector4f& qv
) const noexcept
{
    const float cd = m_params.lin_drag_c;
    const float m = m_params.mass;
    const float T = get_thrust();

    const auto& I = m_params.moment_of_inertia;
    const float Ix = I(0, 0), Iy = I(1, 1), Iz = I(2, 2);
    
    const float qw = qv(0), qx = qv(1), qy = qv(2), qz = qv(3);
    const float wx = w(0), wy = w(1), wz = w(2);

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
            // TODO: Calculate mass based on the requried thrust for takeoff
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

bool copter::handle_command(const pb::Command& command) noexcept
{
    if (!command.has_copter_command()) {
        return false;
    }

    // Result of command execution
    bool command_status = false;
    const pb::vehicles::CopterCommand& copter_command = command.copter_command();
    
    switch (copter_command.command_type_case()) {
    case pb::vehicles::CopterCommand::kSetAngularVelocity:
        const auto& w = copter_command.set_angular_velocity().angular_velocity();
        const float thrust = copter_command.set_angular_velocity().thrust();
        command_status = m_controller.set_target_w({w.x(), w.y(), w.z()}, thrust);
        break;
    case pb::vehicles::CopterCommand::kSetLinearVelocity:
        const auto& v = copter_command.set_linear_velocity().velocity();
        const float dir = copter_command.set_linear_velocity().direction();
        command_status = m_controller.set_target_v({v.x(), v.y(), v.z()}, dir);
        break;
    
    default:
        command_status = false;
    }
    return command_status;
}

}