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
    
}