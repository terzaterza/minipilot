#pragma once

#include "emblib/common/logger.hpp"

#define MP_LOGGER_USE_PROTOBUF      0

namespace mp {

/**
 * Must match with log.proto log_level enum
 */
using emblib::log_level_e;

// Maximum string size in characters (bytes)
static constexpr size_t LOGGER_MAX_INPUT_SIZE = 110;
// String size + prefix and suffix
static constexpr size_t LOGGER_MAX_TOTAL_SIZE = LOGGER_MAX_INPUT_SIZE + 16;

/**
 * Minipilot logger
 * 
 * Converts messages to a protobuf log message format
 * and sends them to a logging char dev
 */
class logger : public emblib::logger<LOGGER_MAX_INPUT_SIZE> {

public:
    static logger& get_instance() noexcept;

private:
    // Singleton
    logger() : emblib::logger<LOGGER_MAX_INPUT_SIZE>(nullptr) {}

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