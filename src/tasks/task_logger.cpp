#include "task_logger.hpp"
#include <cstring>

namespace mp {

ssize_t task_logger::write(const char* data, size_t size, milliseconds_t timeout) noexcept
{
    UNUSED(timeout);
    log_msg_s msg;
    msg.length = size;
    memcpy(msg.data, data, size);
    return m_log_msg_queue.send(msg) ? size : -1;
}

void task_logger::run() noexcept
{
    assert(m_log_device.probe(milliseconds_t(0)));
    bool use_async = m_log_device.is_async_available();

    log_msg_s recv_msg;
    while (true) {
        // TODO: Bypass this copying by reading the queue's top element data
        // and removing the item from queue after write somehow
        m_log_msg_queue.receive(recv_msg);

        if (use_async) {
            m_log_device.write_async(recv_msg.data, recv_msg.length, [this](ssize_t status) {
                notify_from_isr();
            });
            // TODO: FIX: Only wait for notification if write_async started correctly (returned true)
            wait_notification();
        } else {
            m_log_device.write(recv_msg.data, recv_msg.length, milliseconds_t(0));
        }
    }
}

}