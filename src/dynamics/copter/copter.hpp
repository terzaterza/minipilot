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

class copter : public model {

public:
    explicit copter(const copter_params_s& params) noexcept :
        m_params(params)
    {}

    /**
     * Returns the acceleration of the model in the global coordinate frame
     * assuming that thrust is produced in the model::UP direction
     */
    emblib::vector3f acc(const state_s& state) const noexcept override;
    
    emblib::vector3f ang_acc(const state_s& state) const noexcept override;

    void control(const state_s& state) noexcept override;

private:
    /**
     * This method should convert the given thrust and torque values
     * into motor speeds and write those parameters to the motors
     */
    virtual void actuate(float thrust, const emblib::vector3f& torque) noexcept = 0;

private:
    const copter_params_s& m_params;

    /**
     * This values are computed in the control iteration once the
     * desired thrust and torque are calculated as the lerp between
     * the previous values and the new ones
     * @todo Instead can implement 
     * virtual pair<float, vec3f> compute_thrust_and_torque()
     * which would read the current motor speed values and
     * convert them to the currently produced thrust and torque
    */
    float m_approx_current_thrust;
    emblib::vector3f m_approx_current_torque = {0, 0, 0};
};

}