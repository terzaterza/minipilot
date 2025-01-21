#pragma once

#include "dynamics/model.hpp"

namespace mp {

// Structure containing all parameters describing an abstract copter model
struct copter_params_s {
    // Mass of the aircraft in kilograms
    float mass;
    // Inertia matrix (tensor) - usually a diagonal matrix
    matrix3f inertia_matrix;
    // Linear drag coefficient - (can be different for each direction)
    vector3f lin_drag_c;
};

/**
 * A copter is a vehicle which can generate thrust in a single
 * direction (currently fixed as `UP`) and can generate torque
 * in any direction. This model should then be extended to
 * implement actuator control based on thrust and torque input
 * and vice versa.
 */
class copter : public model {

public:
    explicit copter(const copter_params_s& params) noexcept :
        m_params(params)
    {}

    /**
     * Returns the acceleration of the model in the global coordinate frame
     * assuming that thrust is produced in the model::UP direction
     */
    vector3f get_linear_acceleration(
        const vector3f& linear_velocity,
        const quaternionf& rotation_q
    ) const noexcept override;

    /**
     * 
     */
    vector3f get_angular_acceleration(
        const vector3f& linear_velocity,
        const vector3f& angular_velocity,
        const quaternionf& rotation_q
    ) const noexcept override;

    /**
     * 
     */
    jacobian_s get_jacobian(
        const vector3f& linear_velocity,
        const vector3f& angular_velocity,
        const vector4f& rotation_q
    ) const noexcept override;

    /**
     * 
     */
    void control(const state_s& state, float dt) noexcept override;

protected:
    // Container for thrust and torque values
    // Can rename to thrust_torque_s
    struct actuation_s {
        // Thrust magnitude in the up direction in Newtons [N]
        float thrust;
        // Torque vector in Newton-meters [Nm]
        vector3f torque;
    };

private:
    /**
     * This method should convert the given thrust and torque values
     * into motor speeds and write those parameters to the motors
     */
    virtual void actuate(const actuation_s& actuation) noexcept = 0;

    /**
     * Copter implementation should return the currently produced thrust
     * and torque based on motor speeds and appropriate propeller coefficients
     */
    virtual actuation_s get_actuation() const noexcept = 0;

private:
    // Implemented as a reference because it is expected that the
    // parameters can change over time.
    // Alternative to this is to implement the parameters as virtual methods
    const copter_params_s& m_params;
};

}