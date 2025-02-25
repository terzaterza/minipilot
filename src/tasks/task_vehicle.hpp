#pragma once

#include "task_config.hpp"
#include "vehicles/vehicle.hpp"
#include "task_receiver.hpp"
#include "task_state_estimator.hpp"

namespace mp {

/**
 * Task responsible for running the vehicle update iterations
 * and passing the received commands to the vehicle for processing
 * @todo Should be the only task with a reference to the vehicle
 */
class task_vehicle : public emblib::task {

public:
    explicit task_vehicle(
        vehicle& vehicle,
        task_receiver& task_receiver,
        task_state_estimator& task_state_estimator
    ) noexcept;

private:
    void run() noexcept override;

private:
    emblib::task_stack_t<TASK_VEHICLE_STACK_SIZE> m_task_stack;
    vehicle& m_vehicle;

    task_receiver& m_task_receiver;
    task_state_estimator& m_task_state_estimator;

    // Command receive arena
    google::protobuf::Arena m_arena;
    char m_arena_buffer[COMMAND_MSG_MAX_SIZE];
};

}