#pragma once

#include "mp/util/math.hpp"
#include "pb/types.pb.h"

namespace mp {

inline void set_pb_vector3f(pb::Vector3f* pb_vec, const vector3f& mp_vec)
{
    pb_vec->set_x(mp_vec(0));
    pb_vec->set_y(mp_vec(1));
    pb_vec->set_z(mp_vec(2));
}

inline void set_pb_vector4f(pb::Vector4f* pb_vec, const vector4f& mp_vec)
{
    pb_vec->set_w(mp_vec(0));
    pb_vec->set_x(mp_vec(1));
    pb_vec->set_y(mp_vec(2));
    pb_vec->set_z(mp_vec(3));
}

/**
 * Wrapper for a protobuf Arena with static allocation
 */
template <size_t SIZE>
class static_arena {

public:
    static_arena() :
        m_arena(google::protobuf::ArenaOptions {
            .max_block_size = SIZE,
            .initial_block = m_buffer,
            .initial_block_size = SIZE
        })
    {}

    google::protobuf::Arena& get_arena() const noexcept
    {
        return m_arena;
    }

private:
    google::protobuf::Arena m_arena;
    char m_buffer[SIZE];
};
    
}