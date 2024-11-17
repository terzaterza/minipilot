#pragma once

#include "mp_config.hpp"

#include "emblib/rtos/task.hpp"
#include "emblib/rtos/queue.hpp"
#include "emblib/driver/io/char_dev.hpp"
#include "etl/string.h"

namespace mp {

class task_logger : public emblib::task, public emblib::char_dev {

    using log_msg_t = etl::string<LOGGER_MSG_BUFFER_SIZE>;

public:
    explicit task_logger(emblib::char_dev& log_device) :
        task("Task logger", TASK_LOGGER_PRIORITY, m_task_stack),
        m_log_device(log_device)
    {}

    /**
     * Char dev write interface override
     * @note Don't use this as it involves double copying, use `write(const log_msg_t&)`
     */
    ssize_t write(const char* data, size_t size) noexcept override
    {
        log_msg_t temp(data, size);
        return write(temp);
    }

    /**
     * Reading not supported for the log task
     */
    ssize_t read(char* buffer, size_t size) noexcept override
    {
        UNUSED(buffer);
        UNUSED(size);
        return -1;
    }

    /**
     * Overload for when we know that we are writing to a task_logger
     * from the logger class, to bypass double copying
     */
    ssize_t write(const log_msg_t& msg) noexcept
    {
        return m_log_msg_queue.send(msg) ? msg.size() : -1;
    }

private:
    /**
     * Task thread
     */
    void run() noexcept override;

private:
    emblib::queue<log_msg_t, TASK_LOGGER_QUEUE_SIZE> m_log_msg_queue;
    emblib::task_stack_t<TASK_LOGGER_STACK_SIZE> m_task_stack;
    emblib::char_dev& m_log_device;

};

}