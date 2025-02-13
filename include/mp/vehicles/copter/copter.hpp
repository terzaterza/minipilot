#pragma once

#include "mp/vehicles/vehicle.hpp"

namespace mp {

// Structure containing all parameters describing an abstract copter model
struct copter_params_s {
    // Mass of the aircraft in kilograms
    float mass;
    // Inertia matrix (tensor) - usually a diagonal matrix
    matrix3f moment_of_inertia;
    // Linear drag coefficient
    float lin_drag_c;
};

/**
 * A copter is a vehicle which can generate thrust in a single
 * direction (currently fixed as `UP`) and can generate torque
 * in any direction. This model should then be extended to
 * implement actuator control based on thrust and torque input
 * and vice versa (quadcopter, helicopter, ...).
 */
class copter : public vehicle {

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
    void update(const state_s& state, float dt) noexcept override;

private:
    /**
     * This method should convert the given thrust and torque values
     * into motor speeds and write those parameters to the motors
     */
    virtual void actuate(float thrust, const vector3f& torque) noexcept = 0;

    /**
     * Copter implementation should return the currently produced thrust
     * based on motor speeds and appropriate propeller coefficients
     */
    virtual float get_thrust() const noexcept = 0;

    /**
     * Copter implementation should return the currently produced torque
     * based on motor speeds and appropriate propeller coefficients
     */
    virtual vector3f get_torque() const noexcept = 0;

private:
    // Parameters describing a generic copter vehicle
    const copter_params_s& m_params;
};

}