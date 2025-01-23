#pragma once

#include "mp_config.hpp"
#include "task_accelerometer.hpp"
#include "task_gyroscope.hpp"
#include "task_state_estimator.hpp"
#include "emblib/driver/io/char_dev.hpp"
#include "emblib/rtos/task.hpp"
#include "emblib/rtos/queue.hpp"
#include "pb/telemetry.pb.h"

namespace mp {

class task_telemetry : public emblib::task {

public:
    explicit task_telemetry(
        emblib::char_dev& telemetry_device,
        task_accelerometer& task_accelerometer,
        task_gyroscope& task_gyroscope,
        task_state_estimator& task_state_estimator
    ) :
        task("Task telemetry", TASK_TELEMETRY_PRIORITY, m_task_stack),
        m_telemetry_device(telemetry_device),
        m_task_accel(task_accelerometer),
        m_task_gyro(task_gyroscope),
        m_task_state(task_state_estimator),
        m_arena(google::protobuf::ArenaOptions {
            .max_block_size = TASK_TELEMETRY_ARENA_SIZE,
            .initial_block = m_arena_buffer,
            .initial_block_size = TASK_TELEMETRY_ARENA_SIZE
        })
    {}

private:
    void run() noexcept override;

private:
    emblib::task_stack_t<TASK_TELEMETRY_STACK_SIZE> m_task_stack;
    emblib::char_dev& m_telemetry_device;

    task_accelerometer& m_task_accel;
    task_gyroscope& m_task_gyro;
    task_state_estimator& m_task_state;

    char m_arena_buffer[TASK_TELEMETRY_ARENA_SIZE];
    char m_out_msg_buffer[TASK_TELEMETRY_ARENA_SIZE];
    google::protobuf::Arena m_arena;

};

}