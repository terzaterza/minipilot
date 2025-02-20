#pragma once

#include "mp/util/math.hpp"
#include "mp/vehicles/state.hpp"
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
     * Information about the sensor configurations onboard the vehicle
     * @todo Make the vehicle able to provide the list of all onboard
     * sensors and create the appropriate sensor task for each of those
     * and modify the state estimator to use all the appropriate sensors
     */
    struct sensor_config_s {
        // Map the accelerometer readings to the body coordinate frame
        matrix3f accelerometer_transform;
        // Map the gyroscope readings to the body coordinate frame
        matrix3f gyroscope_transform;
        
        // vector3f accelerometer_com_displacement;
    };

public:
    /**
     * Runs once at the beginning of the vehicle task
     * @todo Provide the state as a parameter
     */
    virtual void init() noexcept = 0;

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
     * TODO: View `sensor_config_s` description
     */
    virtual const sensor_config_s& get_sensor_config() const noexcept = 0;

};

}