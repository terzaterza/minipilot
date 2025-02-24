#pragma once

#include "task_config.hpp"
#include "util/logger.hpp"
#include "emblib/driver/io/char_dev.hpp"
#include "emblib/rtos/task.hpp"
#include "emblib/rtos/queue.hpp"

namespace mp {

class task_logger : public emblib::task, public emblib::char_dev {

public:
    explicit task_logger(emblib::char_dev& log_device) :
        task("Task logger", TASK_LOGGER_PRIORITY, m_task_stack),
        m_log_device(log_device)
    {}

    /**
     * Char dev write interface override
     */
    ssize_t write(const char* data, size_t size) noexcept override;

    /**
     * Reading not supported for the log task
     */
    ssize_t read(char* buffer, size_t size) noexcept override
    {
        UNUSED(buffer);
        UNUSED(size);
        return -1;
    }

private:
    /**
     * Task thread
     */
    void run() noexcept override;

private:
    struct log_msg_s {
        char data[LOGGER_MAX_TOTAL_SIZE];
        size_t length;
    };

    emblib::queue<log_msg_s, TASK_LOGGER_QUEUE_SIZE> m_log_msg_queue;
    emblib::task_stack_t<TASK_LOGGER_STACK_SIZE> m_task_stack;
    emblib::char_dev& m_log_device;

};

}