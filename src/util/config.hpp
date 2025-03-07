#pragma once

#include "emblib/driver/io/char_dev.hpp"
#include "pb/config.pb.h"
#include "util/pb_types.hpp"

namespace mp {

class config_manager {

public:
    bool load(emblib::char_dev& storage_dev) noexcept;
    bool save(emblib::char_dev& storage_dev) const noexcept;

    pb::Configuration& get_config() const noexcept
    {
        return *m_configuration;
    }

private:
    void load_default() noexcept;

private:
    pb::Configuration* m_configuration;
    static_arena<256> m_arena;

};

}