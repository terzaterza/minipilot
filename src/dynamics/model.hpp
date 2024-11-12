#pragma once

#include "emblib/math/vector.hpp"
#include "./state.hpp"

namespace mp {

class model {

public:
    static inline const emblib::vector3f FORWARD    = {1, 0, 0};
    static inline const emblib::vector3f LEFT       = {0, 1, 0};
    static inline const emblib::vector3f UP         = {0, 0, 1};
    static inline const emblib::vector3f BACKWARD   = -FORWARD;
    static inline const emblib::vector3f RIGHT      = -LEFT;
    static inline const emblib::vector3f DOWN       = -UP;

public:
    /**
     * Calculate the model's expected acceleration based on the current
     * state in the global reference frame in [m/s^2]
     * @todo Can add typedef for vectors in the global and local reference frames
     */
    virtual emblib::vector3f acc(const state_s& state) const noexcept = 0;

    /**
     * Calculate the model's expected angular acceleration based on the
     * current state in [rad/s^2]
     */
    virtual emblib::vector3f ang_acc(const state_s& state) const noexcept = 0;

    /**
     * Run an iteration of the control algorithm of the model
     * @note This method should control the actuators of the model
     */
    virtual void control(const state_s& state) noexcept = 0;

    
    /** @todo Add process_command(command_s& command) */

};

}