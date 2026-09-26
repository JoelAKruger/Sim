#pragma once

#include <box3d/math_functions.h>
#include <math.h>

#include "core/common.h"

// Box3D's own trigonometry is a fast approximation (about 0.1%), good enough inside the
// solver but not for placing geometry. These use libm, which Nix pins, so results are
// still identical on every machine running the same build.

inline b3Quat make_quat_from_axis_angle(v3 unit_axis, f32 radians)
{
    f32 s = sinf(0.5f * radians);
    return b3Quat{{unit_axis.x * s, unit_axis.y * s, unit_axis.z * s}, cosf(0.5f * radians)};
}
