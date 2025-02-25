#pragma once

#include "mp/util/math.hpp"

namespace mp {

/**
 * Interface of an algorithm which produces required thrust and torque
 * for controlling the copter based on target linear or angular velocity
 */
class copter_controller {
public:
    /**
     * Set target angular velocity and thrust
     * @note Switches the control mode of the copter to angular
     * velocity control if was previously velocity controlled
     * @returns `true` if targets were set successfully
     * @note Can return false in case that target angular velocity or
     * thrust is out of range for example
     */
    virtual bool set_target_w(const vector3f& target_w, float target_thrust) noexcept = 0;

    /**
     * Set the target velocity in the global frame and the direction in radians
     * with NORTH being 0 degrees, increasing CW
     */
    virtual bool set_target_v(const vector3f& target_v, float direction) noexcept = 0;
    
    /**
     * Update the algorithm
     */
    virtual void update(const state_s& state, float dt) noexcept = 0;

    /**
     * Get the output torque of the control algorithm
     */
    virtual vector3f get_torque() const noexcept = 0;

    /**
     * Get the output thrust of the control algorithm
     */
    virtual float get_thrust() const noexcept = 0;
};

}