#include "task_telemetry.hpp"
#include "util/pb_types.hpp"
#include "pb/telemetry.pb.h"

namespace mp {


void task_telemetry::run() noexcept
{
    // This doesn't have to be an assert
    // Can just exit and turn off the telemetry task
    assert(m_telemetry_device.probe());

    while (true) {
        pb::TelemetryMessage* msg = m_arena.Create<pb::TelemetryMessage>(&m_arena);

        // State data
        state_s state = m_task_state.get_state();
        set_pb_vector3f(msg->mutable_state()->mutable_position(), state.position);
        set_pb_vector3f(msg->mutable_state()->mutable_velocity(), state.velocity);
        set_pb_vector3f(msg->mutable_state()->mutable_acceleration(), state.acceleration);
        set_pb_vector3f(msg->mutable_state()->mutable_angular_velocity(), state.angular_velocity);
        set_pb_vector4f(msg->mutable_state()->mutable_rotation(), state.rotationq.as_vector());
        
        // Sensor data
        set_pb_vector3f(msg->mutable_sensor_data()->mutable_acc_raw(), m_task_accel.get_raw());
        set_pb_vector3f(msg->mutable_sensor_data()->mutable_acc_filtered(), m_task_accel.get_filtered());
        set_pb_vector3f(msg->mutable_sensor_data()->mutable_gyro_raw(), m_task_gyro.get_raw());
        set_pb_vector3f(msg->mutable_sensor_data()->mutable_gyro_filtered(), m_task_gyro.get_raw());

        // TODO: Send vehicle specific telemetry here

        // Try to serialize, if successful, transmit
        if (msg->SerializeToArray(m_out_msg_buffer, sizeof(m_out_msg_buffer))) {
            size_t msg_size = msg->ByteSizeLong();

            if (m_telemetry_device.is_async_available()) {
                m_telemetry_device.write_async(m_out_msg_buffer, msg_size, [this](ssize_t status) {
                    notify();
                });
                wait_notification();
            } else {
                m_telemetry_device.write(m_out_msg_buffer, msg_size);
            }
        }

        // Clear the protobuf allocation buffer
        m_arena.Reset();
        sleep_periodic(TASK_TELEMETRY_PERIOD);
    }
}

}