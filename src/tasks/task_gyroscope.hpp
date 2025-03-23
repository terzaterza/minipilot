#pragma once

#include "task_config.hpp"
#include "task_three_axis_sensor.hpp"
#include "emblib/driver/sensor/gyroscope.hpp"

namespace mp {

// TODO: Replace float data_type with rad/s
class task_gyroscope : public task_three_axis_sensor<float> {

public:
    explicit task_gyroscope(
        emblib::gyroscope& gyroscope,
        matrix_t transform
    );

private:
    vector_t process(const vector_t& raw_data) const noexcept override;

private:
    matrix_t m_transform;
    // TODO: Add band-pass filter
};

}