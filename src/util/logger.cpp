#include "./logger.hpp"
#include "pb/log.pb.h"

namespace mp {

logger& logger::get_instance() noexcept
{
    static logger s_logger;
    return s_logger;
}

#if MP_LOGGER_USE_PROTOBUF

static char g_arena_buffer[LOGGER_MSG_BUFFER_SIZE];

static google::protobuf::ArenaOptions g_arena_options {
    .initial_block = g_arena_buffer,
    .initial_block_size = LOGGER_MSG_BUFFER_SIZE,
    .max_block_size = LOGGER_MSG_BUFFER_SIZE
};
static google::protobuf::Arena g_logger_arena(g_arena_options);

/**
 * This function is running while the logger mutex is being
 * held so there is no conflict over arena ownership
 */
void logger::flush(log_level_e level) noexcept
{
    pb::log_message* msg = g_logger_arena.CreateMessage<pb::log_message>(&g_logger_arena);
    msg->set_level(static_cast<pb::log_level_e>(level));
    msg->set_message(m_buffer.c_str());

    static char serialized_msg[LOGGER_MSG_BUFFER_SIZE];
    static size_t serailized_size;

    if (msg->SerializeToArray(serialized_msg, serailized_size)) {
        m_log_device->write(serialized_msg, serailized_size);
        m_buffer.clear();
        /** @todo Instead of clear can remove number of written
         * bytes from the beginning of the buffer */
    }

    g_logger_arena.Reset();
}

#else

void logger::flush(log_level_e level, const buffer_t& buffer, emblib::char_dev& log_device) noexcept
{
    // Using the same size for the formatted message as for the protobuf approximately
    static etl::string<LOGGER_MAX_TOTAL_SIZE> formatted_msg_buffer;
    
    static const char* level_prefix[] = {"DEBUG", "INFO", "WARNING", "ERROR"};
    formatted_msg_buffer = level_prefix[static_cast<int>(level)];
    formatted_msg_buffer += ": ";
    formatted_msg_buffer += buffer;
    formatted_msg_buffer += "\n";
    
    log_device.write(formatted_msg_buffer.c_str(), formatted_msg_buffer.size());
    formatted_msg_buffer.clear();
}

#endif

}