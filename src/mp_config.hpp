#pragma once

#include <assert.h>
#include <cstddef>
#include <chrono>

#define MP_LOGGER_USE_PROTOBUF      0

namespace mp {

enum task_priority_e : size_t {
    TASK_PRIORITY_VERY_LOW  = 1,
    TASK_PRIORITY_LOW       = 2,
    TASK_PRIORITY_MEDIUM    = 3,
    TASK_PRIORITY_HIGH      = 4,
    TASK_PRIORITY_REALTIME  = 5
};

// Stack and buffer sizes are in bytes

static constexpr size_t             LOGGER_MAX_STR_SIZE         = 128;
static constexpr size_t             LOGGER_MSG_BUFFER_SIZE      = LOGGER_MAX_STR_SIZE + 16; // Approximate addition
static constexpr size_t             COMMAND_MSG_MAX_SIZE        = 64;

static constexpr size_t             TASK_LOGGER_QUEUE_SIZE      = 8;
static constexpr size_t             TASK_LOGGER_STACK_SIZE      = 1024;
static constexpr task_priority_e    TASK_LOGGER_PRIORITY        = TASK_PRIORITY_VERY_LOW;

static constexpr size_t             TASK_TELEMETRY_STACK_SIZE   = 1024;
static constexpr size_t             TASK_TELEMETRY_ARENA_SIZE   = 256;
static constexpr task_priority_e    TASK_TELEMETRY_PRIORITY     = TASK_PRIORITY_LOW;
static constexpr auto               TASK_TELEMETRY_PERIOD       = std::chrono::milliseconds(200); // 5Hz

static constexpr size_t             TASK_ACCEL_STACK_SIZE       = 512;
static constexpr task_priority_e    TASK_ACCEL_PRIORITY         = TASK_PRIORITY_REALTIME;
static constexpr auto               TASK_ACCEL_PERIOD           = std::chrono::milliseconds(5); // 200Hz

static constexpr size_t             TASK_GYRO_STACK_SIZE        = 512;
static constexpr task_priority_e    TASK_GYRO_PRIORITY          = TASK_PRIORITY_REALTIME;
static constexpr auto               TASK_GYRO_PERIOD            = std::chrono::milliseconds(5); // 200Hz

static constexpr size_t             TASK_STATE_STACK_SIZE       = 24576;
static constexpr task_priority_e    TASK_STATE_PRIORITY         = TASK_PRIORITY_REALTIME;
static constexpr auto               TASK_STATE_PERIOD           = std::chrono::milliseconds(20); // 50Hz

static constexpr size_t             TASK_RECEIVER_STACK_SIZE    = 1024;
static constexpr size_t             TASK_RECEIVER_QUEUE_SIZE    = 4;
static constexpr size_t             TASK_RECEIVER_ARENA_SIZE    = TASK_RECEIVER_QUEUE_SIZE * COMMAND_MSG_MAX_SIZE;
static constexpr task_priority_e    TASK_RECEIVER_PRIORITY      = TASK_PRIORITY_HIGH;

static constexpr size_t             TASK_VEHICLE_STACK_SIZE     = 4096;
static constexpr task_priority_e    TASK_VEHICLE_PRIORITY       = TASK_PRIORITY_HIGH;
static constexpr auto               TASK_VEHICLE_PERIOD         = std::chrono::milliseconds(50); // 20Hz

}