#include "task_accelerometer.hpp"

namespace mp {

task_accelerometer::task_accelerometer(
    emblib::accelerometer& accelerometer,
    matrix_t transform,
    vector_t bias
) :
    task_three_axis_sensor(
        accelerometer,
        "Task accelerometer",
        TASK_ACCEL_PRIORITY,
        TASK_ACCEL_PERIOD
    ),
    m_bias(bias),
    m_transform(transform)
{}

task_accelerometer::vector_t
task_accelerometer::process(const vector_t& raw_data) const noexcept
{
    // TODO: Add (conditional) low-pass filtering
    return m_transform.matmul(raw_data - m_bias);
}

}