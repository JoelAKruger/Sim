#pragma once

#include "core/allocator.h"
#include "core/config.h"

// Height grid shared by physics, rendering and sensors. The world frame is REP-103
// (x east, y north, z up). Sample (row, col) lies at
//     x = origin_x + col * spacing
//     y = origin_y - row * spacing
// so row 0 is the north edge, the same way round as a north-up heightmap image.
struct Terrain {
    f32 *heights; // rows * cols, row-major, metres
    u32 rows;
    u32 cols;
    f32 spacing;
    f32 origin_x; // world position of sample (0, 0), the north-west corner
    f32 origin_y;
    f32 min_height;
    f32 max_height;
};

// Rolling hills plus craters, deterministic for a given seed. The spawn area near the
// origin is kept free of craters.
bool generate_terrain(Terrain *terrain, Linear_Allocator *allocator, const Sim_Config *config);

// Height at world (x, y), interpolated over the two triangles per cell that the physics
// collides with (split along the south-west to north-east diagonal), so it agrees with
// physics exactly. Clamped to the grid edges. Works on any grid, the soil's too.
f32 get_terrain_height(const Terrain *terrain, f32 x, f32 y);

// A binary PGM (P5), north up, one sample per pixel, spaced terrain.spacing_m apart and
// centred on the origin. The full pixel range spans terrain.height_m. 16-bit samples are
// best; 8-bit ones load with a warning, since the steps are coarse enough to feel.
bool load_heightmap(Terrain *terrain, Linear_Allocator *allocator, const Sim_Config *config,
                    char *error, u32 error_size);
