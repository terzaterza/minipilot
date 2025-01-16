#pragma once

#include "dynamics/model.hpp"

namespace mp {

struct copter_params_s {
    // Mass of the aircraft in kilograms
    float mass;
    // Inertia matrix (tensor) - usually a diagonal matrix
    emblib::matrix<float, 3, 3> inertia_matrix;
    // Linear drag coefficient
    emblib::vector3f lin_drag_c;
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
    vector3f acc(const vector3f& vel, const quaternionf& rotq) const noexcept override;

    /**
     * 
     */
    vector3f ang_acc(const vector3f& vel, const quaternionf& rotq, const vector3f& ang_vel) const noexcept override;

    /**
     * 
     */
    jacobian_s jacobian(const vector3f& vel, const vector4f& rotqv, const vector3f& ang_vel) const noexcept override;

    /**
     * 
     */
    void control(const state_s& state, const actuators_s& actuators) noexcept override;

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
    virtual float thrust() const noexcept = 0;

    /**
     * Current torque produced by the copter based on motor speeds, ...
     */
    virtual vector3f torque() const noexcept = 0;

private:
    // Implemented as a reference because it is expected that the
    // parameters can change over time.
    // Alternative to this is to implement the parameters as virtual methods
    const copter_params_s& m_params;
};

}