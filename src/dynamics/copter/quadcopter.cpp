#include "quadcopter.hpp"
#include <cmath>

namespace mp {

void quadcopter::actuate(const actuation_s& actuation) noexcept
{
    motor_speeds_s required_speeds = inverse_mma(actuation);

    // Assuming that the writes will not fail
    // TODO: Handle write failure
    m_actuators.fl.write_speed(required_speeds.fl);
    m_actuators.fr.write_speed(required_speeds.fr);
    m_actuators.bl.write_speed(required_speeds.bl);
    m_actuators.br.write_speed(required_speeds.br);
}

quadcopter::actuation_s quadcopter::get_actuation() const noexcept
{
    motor_speeds_s current_speeds = read_motor_speeds();

    const float w_fl_sq = current_speeds.fl * current_speeds.fl;
    const float w_fr_sq = current_speeds.fr * current_speeds.fr;
    const float w_bl_sq = current_speeds.bl * current_speeds.bl;
    const float w_br_sq = current_speeds.br * current_speeds.br;

    float thrust = m_params.thrust_coeff * (w_fl_sq + w_fr_sq + w_bl_sq + w_br_sq);
    
    // Signs of the motor speeds depend on the configuration
    // TODO: Make configurable if torque_up needs to be multiplied by -1
    float torque_up = m_params.torque_coeff * (w_fl_sq - w_fr_sq - w_bl_sq + w_br_sq);

    const float fwd_coeff = m_params.thrust_coeff * m_params.arm_length * std::cos(m_params.arm_angle / 2.f);
    const float left_coeff = m_params.thrust_coeff * m_params.arm_length * std::sin(m_params.arm_angle / 2.f);
    float torque_fwd = fwd_coeff * (w_fl_sq + w_bl_sq - w_fr_sq - w_br_sq);
    float torque_left = left_coeff * (w_bl_sq + w_br_sq - w_fl_sq - w_fr_sq);

    vector3f torque = UP * torque_up + FORWARD * torque_fwd + LEFT * torque_left;
    
    return {thrust, torque};
}



}