#include "task_telemetry.hpp"
#include "pb/telemetry.pb.h"

namespace mp {

static void vector3f_to_pb(const emblib::vector3f& vec_in, pb::Vector3f& vec_out)
{
    vec_out.set_x(vec_in(0));
    vec_out.set_y(vec_in(1));
    vec_out.set_z(vec_in(2));
}

static void vector4f_to_pb(const emblib::vectorf<4>& vec_in, pb::Vector4f& vec_out)
{
    vec_out.set_w(vec_in(0));
    vec_out.set_x(vec_in(1));
    vec_out.set_y(vec_in(2));
    vec_out.set_z(vec_in(3));
}

void task_telemetry::run() noexcept
{
    // This doesn't have to be an assert
    // Can just exit and turn off the telemetry task
    assert(m_telemetry_device.probe());

    while (true) {
        pb::TelemetryMessage* msg = m_arena.Create<pb::TelemetryMessage>(&m_arena);
        
        // Raw sensor readings
        auto accel_raw = m_task_accel.get_raw();
        auto gyro_raw = m_task_gyro.get_raw();
        vector3f_to_pb(accel_raw, *msg->mutable_acc_raw());
        vector3f_to_pb(gyro_raw, *msg->mutable_gyro_raw());

        // State data
        auto state = m_task_state.get_state();
        vector3f_to_pb(state.position, *msg->mutable_position());
        vector3f_to_pb(state.velocity, *msg->mutable_velocity());
        vector4f_to_pb(state.rotationq.as_vector(), *msg->mutable_rotation());
        vector3f_to_pb(state.angular_velocity, *msg->mutable_ang_velocity());
        msg->set_grounded(state.grounded);

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