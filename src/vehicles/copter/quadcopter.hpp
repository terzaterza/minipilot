#pragma once

#include "vehicles/copter/copter.hpp"
#include "emblib/driver/actuator/motor.hpp"

namespace mp {

struct quadcopter_params_s : public copter_params_s {
    // Half of the width of the quad measured from the centers of left and right motors
    float width_half;
    // Half of the length of the quad measured from the centers of front and back motors
    float length_half;
    // Thrust at max throttle (assuming T = thrust_coeff * throttle^2)
    float thrust_coeff;
    // Torque at max throttle (assuming Tau = torque_coeff * throttle^2)
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
    explicit quadcopter(const quadcopter_params_s& params, copter_controller& controller, quadcopter_actuators_s actuators) noexcept :
        copter(params, controller), m_params(params), m_actuators(actuators)
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

    /**
     * Get the direction of each motor, setting 1 if CCW else -1
     */
    motor_speeds_s get_motor_directions() const noexcept;

private:
    const quadcopter_params_s& m_params;
    quadcopter_actuators_s m_actuators;
};

}