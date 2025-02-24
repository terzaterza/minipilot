#pragma once

#include "task_config.hpp"
#include "pb/command.pb.h"
#include "emblib/driver/io/char_dev.hpp"
#include "emblib/rtos/task.hpp"
#include "emblib/rtos/queue.hpp"

namespace mp {

class task_receiver : public emblib::task {
public:
    task_receiver(emblib::char_dev& receiver_device) noexcept;

    /**
     * Returns true if a command was available and was successfully copied
     * into the provided buffer
     */
    bool get_command(pb::Command* command_buffer) noexcept;

private:
    void run() noexcept override;

private:
    emblib::task_stack_t<TASK_RECEIVER_STACK_SIZE> m_task_stack;
    emblib::char_dev& m_receiver_device;
    
    emblib::queue<pb::Command*, TASK_RECEIVER_QUEUE_SIZE> m_command_queue;
    char m_arena_buffer[TASK_RECEIVER_ARENA_SIZE];
    char m_recv_buffer[COMMAND_MSG_MAX_SIZE];
    google::protobuf::Arena m_arena;
};

}