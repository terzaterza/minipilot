#pragma once

#include "mp/util/math.hpp"
#include "state/state_estimator.hpp"
#include "pb/command.pb.h"

namespace mp {

/**
 * Base class for all vehicles
 * 
 * Defines the interface via which the rest of the
 * system talks to different vehicles
 */
class vehicle {

public:
    /**
     * Runs once at the beginning of the vehicle task
     * @todo Provide the state as a parameter
     */
    virtual bool init() noexcept = 0;

    /**
     * Update the internal state (control algorithm for example)
     */
    virtual void update(const state_s& state, float dt) noexcept = 0;

    /**
     * @returns false if the command is not for this vehicle type
     */
    virtual bool handle_command(const pb::Command& command) noexcept = 0;

    /**
     * Get information about onboard sensors
     * @note Should provide a list of all available sensors and a task
     * should be created for each one. The state estimator can have a
     * virtual method to fetch all the required sensor data and use it
     * to call the state estimator's update iteration
     */
    // virtual const sensor_config_s& get_sensor_config() const noexcept = 0;

    /// @todo Add `virtual void navigate(const coords_s& coords, float dt)`
    /// where coords are provided by the state estimator task based on both
    /// gps data but also corrected using the ins velocity integration

};

}