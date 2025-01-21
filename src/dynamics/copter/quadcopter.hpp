#pragma once

#include "./copter.hpp"
#include "emblib/driver/actuator/motor.hpp"

namespace mp {

struct quadcopter_params_s : public copter_params_s {
    // Distance from the center of mass (rotation) to the motor(s)
    float arm_length;
    // Angle between front and rear motors in radians - PI/2 if symmetric
    float arm_angle;
    // Coefficient converting from RPM^2 to thrust (Newtons)
    float thrust_coeff;
    // Coefficient converting from RPM^2 to torque around center of mass
    float torque_coeff;

    /** @note thrust and torque coeffs are not in copter_params_s since
     * a helicopter could have different values for the main rotor and tail rotor */
};

struct quadcopter_actuators_s {
    emblib::motor &fl, &fr, &bl, &br;
};

class quadcopter : public copter {

    struct motor_speeds_s {
        float fl, fr, bl, br;
    };

public:
    explicit quadcopter(const quadcopter_params_s& params, quadcopter_actuators_s actuators) noexcept :
        copter(params), m_params(params), m_actuators(actuators)
    {}

private:
    /**
     * Computes the needed speeds via inverse_mma and assigns them to the appropriate motors
     */
    void actuate(const actuation_s& actuation) noexcept override;

    /**
     * Compute the thrust and torque based on current motor speeds
     * @note Inverse of the `actuate` method
     */
    actuation_s get_actuation() const noexcept override;

    /**
     * Compute the motor speeds to produce the given thrust and torque
     * @todo Make static and pass the copter params as an arg
     */
    motor_speeds_s inverse_mma(const actuation_s& actuation) const noexcept;

    /**
     * Read the current motor speeds
     * @note Assuming that all motor.read_speed calls are successful
     */
    motor_speeds_s read_motor_speeds() const noexcept
    {
        motor_speeds_s result;
        m_actuators.fl.read_speed(result.fl);
        m_actuators.fr.read_speed(result.fr);
        m_actuators.bl.read_speed(result.bl);
        m_actuators.br.read_speed(result.br);
        return result;
    }

private:
    const quadcopter_params_s& m_params;
    quadcopter_actuators_s m_actuators;
};

}