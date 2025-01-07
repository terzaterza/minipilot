#pragma once

#include "./copter.hpp"

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

class quadcopter : public copter {

    struct motor_speeds_s {
        float fl, fr, bl, br;
    };

public:
    /**
     * @todo Add motor_driver& fl, fr, bl, br to constructor arguments
     */
    explicit quadcopter(const quadcopter_params_s& params) noexcept :
        copter(params), m_params(params)
    {}

private:
    /**
     * Compute the motor speeds to produce the given thrust and torque
     */
    motor_speeds_s inverse_mma(float thrust, const emblib::vector3f& torque) const noexcept;

    /**
     * Computes the needed speeds via inverse_mma and assigns them to the appropriate motors
     */
    void actuate(float thrust, const emblib::vector3f& torque) noexcept override;

private:
    const quadcopter_params_s& m_params;

};

}