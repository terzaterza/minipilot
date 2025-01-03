#pragma once

#include "emblib/driver/sensor/accelerometer.hpp"
#include <cmath>

class dummy_accel : public emblib::accelerometer {
public:
    bool probe() noexcept override
    {
        return true;
    }

    bool is_data_available() noexcept override
    {
        return true;
    }

    bool read_axis(axis_e axis, float& out_g) noexcept
    {
        out_g = rand() / (float)RAND_MAX;
        out_g = out_g * 2.f - 1.f;
        return true;
    }

    float get_noise_density() const noexcept
    {
        return 1.f;
    }
};