#pragma once

#include "task_config.hpp"
#include "task_three_axis_sensor.hpp"
#include "emblib/driver/sensor/accelerometer.hpp"

namespace mp {

// TODO: Replace float data_type with m/s^2
class task_accelerometer : public task_three_axis_sensor<float> {

public:
    explicit task_accelerometer(
        emblib::accelerometer& accelerometer,
        matrix_t transform,
        vector_t bias
    );

private:
    vector_t process(const vector_t& raw_data) const noexcept override;

private:
    vector_t m_bias;
    matrix_t m_transform;
    // TODO: Add low-pass filter
};

}