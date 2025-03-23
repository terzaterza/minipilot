#pragma once

#include <assert.h>
#include <cstddef>
#include <chrono>

namespace mp {

enum task_priority_e : size_t {
    TASK_PRIORITY_VERY_LOW  = 1,
    TASK_PRIORITY_LOW       = 2,
    TASK_PRIORITY_MEDIUM    = 3,
    TASK_PRIORITY_HIGH      = 4,
    TASK_PRIORITY_REALTIME  = 5
};

// Stack and buffer sizes are in bytes

inline constexpr size_t             COMMAND_MSG_MAX_SIZE        = 64;

inline constexpr size_t             TASK_LOGGER_QUEUE_SIZE      = 8;
inline constexpr size_t             TASK_LOGGER_STACK_SIZE      = 1024;
inline constexpr task_priority_e    TASK_LOGGER_PRIORITY        = TASK_PRIORITY_VERY_LOW;

inline constexpr size_t             TASK_TELEMETRY_STACK_SIZE   = 1024;
inline constexpr size_t             TASK_TELEMETRY_ARENA_SIZE   = 256;
inline constexpr task_priority_e    TASK_TELEMETRY_PRIORITY     = TASK_PRIORITY_LOW;
inline constexpr auto               TASK_TELEMETRY_PERIOD       = std::chrono::milliseconds(200); // 5Hz

inline constexpr task_priority_e    TASK_ACCEL_PRIORITY         = TASK_PRIORITY_REALTIME;
inline constexpr auto               TASK_ACCEL_PERIOD           = std::chrono::milliseconds(5); // 200Hz

inline constexpr task_priority_e    TASK_GYRO_PRIORITY          = TASK_PRIORITY_REALTIME;
inline constexpr auto               TASK_GYRO_PERIOD            = std::chrono::milliseconds(5); // 200Hz

inline constexpr size_t             TASK_STATE_STACK_SIZE       = 24576;
inline constexpr task_priority_e    TASK_STATE_PRIORITY         = TASK_PRIORITY_REALTIME;
inline constexpr auto               TASK_STATE_PERIOD           = std::chrono::milliseconds(20); // 50Hz

inline constexpr size_t             TASK_RECEIVER_STACK_SIZE    = 1024;
inline constexpr size_t             TASK_RECEIVER_QUEUE_SIZE    = 4;
inline constexpr size_t             TASK_RECEIVER_ARENA_SIZE    = TASK_RECEIVER_QUEUE_SIZE * COMMAND_MSG_MAX_SIZE;
inline constexpr task_priority_e    TASK_RECEIVER_PRIORITY      = TASK_PRIORITY_HIGH;

inline constexpr size_t             TASK_VEHICLE_STACK_SIZE     = 4096;
inline constexpr task_priority_e    TASK_VEHICLE_PRIORITY       = TASK_PRIORITY_HIGH;
inline constexpr auto               TASK_VEHICLE_PERIOD         = std::chrono::milliseconds(50); // 20Hz

}