#pragma once

#include "mp/vehicles/copter/copter.hpp"
#include "emblib/driver/actuator/motor.hpp"

namespace mp {

struct quadcopter_params_s : public copter_params_s {
    // Distance from the center of mass (rotation) to the motor(s)
    float arm_length;
    // Angle between front and rear motors in radians - PI/2 if symmetric
    float arm_angle;
    // Coefficient converting from (rad/s)^2 to thrust (Newtons)
    float thrust_coeff;
    // Coefficient converting from (rad/s)^2 to torque around center of mass
    float torque_coeff;
};

struct quadcopter_actuators_s {
    emblib::motor &fl, &fr, &bl, &br;
};


class quadcopter : public copter {

    // Angular frequency of each motor in rad/s
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
    void actuate(float thrust, const vector3f& torque) noexcept override;

    /**
     * Compute the thrust based on current motor speeds
     * @note Inverse of the `actuate` method
     */
    float get_thrust() const noexcept override;

    /**
     * Compute the torque based on current motor speeds
     * @note Inverse of the `actuate` method
     */
    vector3f get_torque() const noexcept override;

    /**
     * Compute the motor speeds to produce the given thrust and torque
     * @todo Make static and pass the copter params as an arg
     */
    motor_speeds_s inverse_mma(float thrust, const vector3f& torque) const noexcept;

    /**
     * Read the current motor speeds
     * @note Assuming that all motor.read_speed calls are successful
     */
    motor_speeds_s read_motor_speeds(bool square = false) const noexcept;

private:
    const quadcopter_params_s& m_params;
    quadcopter_actuators_s m_actuators;
};

}