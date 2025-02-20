#include "task_receiver.hpp"
#include "util/logger.hpp"

namespace mp {

task_receiver::task_receiver(emblib::char_dev& receiver_device) noexcept :
    task("Task Receiver", TASK_RECEIVER_PRIORITY, m_task_stack),
    m_receiver_device(receiver_device),
    m_arena(google::protobuf::ArenaOptions {
        .max_block_size = TASK_RECEIVER_ARENA_SIZE,
        .initial_block = m_arena_buffer,
        .initial_block_size = TASK_RECEIVER_ARENA_SIZE
    })
{}

bool task_receiver::get_command(pb::Command* command_buffer) noexcept
{
    pb::Command* next_command = nullptr;

    // If the queue is empty just return false
    if (!m_command_queue.receive(next_command, emblib::ticks_t(0)))
        return false;

    // Copy the command to the provided buffer and free up space in this arena
    *command_buffer = *next_command;
    m_arena.Destroy(next_command);
    return true;
}

void task_receiver::run() noexcept
{
    assert(m_receiver_device.is_async_available());

    ssize_t recv_status = 0;
    while (true) {
        // Try to start an async read from the receiver device
        // If the start was unsuccessful, go to sleep, else try to parse the data
        if (m_receiver_device.read_async(m_recv_buffer, COMMAND_MSG_MAX_SIZE, [this, &recv_status](ssize_t status) {
            recv_status = status;
            notify_from_isr();
        })) {
            // If the read was started okay, we're waiting for the notify from the
            // read callback to start the processing
            wait_notification();
            if (recv_status < 0)
                continue;

            pb::Command* command = m_arena.Create<pb::Command>(&m_arena);
            if (command == nullptr) {
                log_warning("Can't create the command in the arena!");
                continue;
            }

            // Try to parse, if okay, put it into the queue
            if (command->ParseFromArray(m_recv_buffer, recv_status)) {
                log_debug("Command received and parsed!");
                // Try to put the command into the queue if there is space
                if (!m_command_queue.send(command, emblib::ticks_t(0)))
                    log_warning("No space in command queue!");
            }
        } else {
            // Starting an async read was not successful, go to sleep for some time
            sleep(std::chrono::milliseconds(100));
        }
    }
}

}