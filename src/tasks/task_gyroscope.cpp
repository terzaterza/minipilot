#include "task_gyroscope.hpp"

namespace mp {

task_gyroscope::task_gyroscope(
    emblib::gyroscope& gyroscope,
    matrix_t transform
) :
    task_three_axis_sensor(
        gyroscope,
        "Task gyroscope",
        TASK_GYRO_PRIORITY,
        TASK_GYRO_PERIOD
    ),
    m_transform(transform)
{}

task_gyroscope::vector_t
task_gyroscope::process(const vector_t& raw_data) const noexcept
{
    // TODO: Add (conditional) band-pass / high-pass filtering
    return m_transform.matmul(raw_data);
}

}