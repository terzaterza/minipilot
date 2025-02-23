#include "task_vehicle.hpp"
#include "util/logger.hpp"

namespace mp {

// Conversion of the task period to floating point delta time
static constexpr float DT = std::chrono::duration<float>(TASK_VEHICLE_PERIOD).count();

task_vehicle::task_vehicle(
    vehicle& vehicle,
    task_receiver& task_receiver,
    task_state_estimator& task_state_estimator
) noexcept :
    task("Task vehicle", TASK_VEHICLE_PRIORITY, m_task_stack),
    m_vehicle(vehicle),
    m_task_receiver(task_receiver),
    m_task_state_estimator(task_state_estimator),
    m_arena(google::protobuf::ArenaOptions {
        .max_block_size = COMMAND_MSG_MAX_SIZE,
        .initial_block = m_arena_buffer,
        .initial_block_size = COMMAND_MSG_MAX_SIZE
    })
{}

void task_vehicle::run() noexcept
{
    // Vehicle's init must complete successfully for
    // the rest of the system to run as intended
    if (!m_vehicle.init()) {
        log_error("Vehicle init failed!");
        assert(false);
    }

    while (true) {
        // Clear any leftover data
        m_arena.Reset();

        // See if there are any commands available and execute them
        // before running the next iteration of the update loop
        while (true) {
            pb::Command* command = m_arena.Create<pb::Command>(&m_arena);
            
            // If no command was available, exit the loop
            if (!m_task_receiver.get_command(command)) {
                break;
            }
            
            // TODO: If false is returned, this command was not for this
            // vehicle, try to handle it globally
            m_vehicle.handle_command(*command);
            m_arena.Destroy(command);
        }
        
        state_s state = m_task_state_estimator.get_state();
        m_vehicle.update(state, DT);

        sleep_periodic(TASK_VEHICLE_PERIOD);
    }
}

}