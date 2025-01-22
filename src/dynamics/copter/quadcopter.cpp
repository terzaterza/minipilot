#include "quadcopter.hpp"
#include <cmath>

namespace mp {

void quadcopter::actuate(float thrust, const vector3f& torque) noexcept
{
    motor_speeds_s required_speeds = inverse_mma(thrust, torque);

    // Assuming that the writes will not fail
    // TODO: Handle write failure
    m_actuators.fl.write_speed(required_speeds.fl);
    m_actuators.fr.write_speed(required_speeds.fr);
    m_actuators.bl.write_speed(required_speeds.bl);
    m_actuators.br.write_speed(required_speeds.br);
}

float quadcopter::get_thrust() const noexcept
{
    motor_speeds_s w_sq = read_motor_speeds(true);
    return m_params.thrust_coeff * (w_sq.fl + w_sq.fr + w_sq.bl + w_sq.br);
}

vector3f quadcopter::get_torque() const noexcept
{
    motor_speeds_s w_sq = read_motor_speeds(true);

    // Signs of the motor speeds depend on the configuration of the CW and CCW propellers
    // TODO: Make configurable if torque_up needs to be multiplied by -1
    float torque_up = m_params.torque_coeff * (w_sq.fl - w_sq.fr - w_sq.bl + w_sq.br);

    const float fwd_coeff = m_params.thrust_coeff * m_params.arm_length * std::cos(m_params.arm_angle / 2.f);
    const float left_coeff = m_params.thrust_coeff * m_params.arm_length * std::sin(m_params.arm_angle / 2.f);
    float torque_fwd = fwd_coeff * (w_sq.fl + w_sq.bl - w_sq.fr - w_sq.br);
    float torque_left = left_coeff * (w_sq.bl + w_sq.br - w_sq.fl - w_sq.fr);

    return UP * torque_up + FORWARD * torque_fwd + LEFT * torque_left;
}

quadcopter::motor_speeds_s quadcopter::inverse_mma(float thrust, const vector3f& torque) const noexcept
{
    return {0, 0, 0, 0};
}

quadcopter::motor_speeds_s quadcopter::read_motor_speeds(bool square) const noexcept
{
    motor_speeds_s result;

    m_actuators.fl.read_speed(result.fl);
    m_actuators.fr.read_speed(result.fr);
    m_actuators.bl.read_speed(result.bl);
    m_actuators.br.read_speed(result.br);

    if (square) {
        result.fl *= result.fl;
        result.fr *= result.fr;
        result.bl *= result.bl;
        result.br *= result.br;
    }
    return result;
}

}