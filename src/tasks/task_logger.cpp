#include "./task_logger.hpp"

#include <algorithm>
#include <cstring>

namespace mp {

void task_logger::run() noexcept
{
    assert(m_log_device.probe());
    bool use_async = m_log_device.is_async_available();

    while (true) {
        /** @todo Bypass this copying by reading with peek and removing from queue later somehow */
        static log_msg_t queue_msg;
        m_log_msg_queue.receive(queue_msg);

        if (use_async) {
            m_log_device.write_async(queue_msg.c_str(), queue_msg.size(), [this](ssize_t status) {
                notify();
            });
            wait_notification();
        } else {
            m_log_device.write(queue_msg.c_str(), queue_msg.size());
        }
    }
}

}