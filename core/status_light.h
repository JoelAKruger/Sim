#pragma once

#include "core/common.h"

// The rover's status light (its LED strip), as shown: each channel 0..1 after brightness.
struct Status_Light {
    f32 red;
    f32 green;
    f32 blue;
};
