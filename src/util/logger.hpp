#pragma once

#include "mp_config.hpp"
#include "emblib/common/logger.hpp"

namespace mp {

/**
 * Must match with log.proto log_level enum
 */
using emblib::log_level_e;

/**
 * Minipilot logger
 * 
 * Converts messages to a protobuf log message format
 * and sends them to a logging char dev
 */
class logger : public emblib::logger<LOGGER_MAX_STR_SIZE> {

public:
    static logger& get_instance() noexcept;

private:
    /* Singleton */
    logger() : emblib::logger<LOGGER_MAX_STR_SIZE>(nullptr) {}

    void flush(log_level_e level, const buffer_t& buffer, emblib::char_dev& log_device) noexcept override;
};

static void log_set_level(log_level_e level) noexcept
{
    logger::get_instance().set_output_level(level);
}

template <typename ...item_types>
static void log_debug(item_types&& ...items) noexcept
{
    logger::get_instance().log(log_level_e::DEBUG, items...);
}

template <typename ...item_types>
static void log_info(item_types&& ...items) noexcept
{
    logger::get_instance().log(log_level_e::INFO, items...);
}

template <typename ...item_types>
static void log_warning(item_types&& ...items) noexcept
{
    logger::get_instance().log(log_level_e::WARNING, items...);
}

template <typename ...item_types>
static void log_error(item_types&& ...items) noexcept
{
    logger::get_instance().log(log_level_e::ERROR, items...);
}

}