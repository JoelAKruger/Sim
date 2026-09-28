#pragma once

#include "core/allocator.h"
#include "core/config.h"
#include "core/terrain.h"

// A patch of deformable soil set into the terrain around the spawn point: a height grid
// much finer than the terrain's, laid out the same way (row 0 north, two triangles per
// cell split the same way), which the physics presses down and heaps up as wheels and
// tools work it. It covers a whole number of terrain cells exactly; the rigid ground has a
// hole there. At the start its surface is the terrain's.

#define SOIL_BLOCK_CELLS 8 // cells per edge of a block with height bounds, for ray casts
#define SOIL_TILE_CELLS 64 // cells per edge of a tile whose changes are counted, for drawing

struct Soil {
    Terrain grid; // the surface now
    // The terrain cells the soil replaces.
    u32 first_row;
    u32 first_col;
    u32 cell_rows;
    u32 cell_cols;
    // Per block of SOIL_BLOCK_CELLS² cells: heights no lower and no higher than any in it.
    f32 *block_low;
    f32 *block_high;
    u32 block_rows;
    u32 block_cols;
    // Per tile of SOIL_TILE_CELLS² cells: bumped whenever a sample in it changes.
    u32 *tile_versions;
    u32 tile_rows;
    u32 tile_cols;
    u64 changes; // samples changed in total
};

// Lays the patch out over the terrain (soil.size_m, centred on robot.spawn) with the
// terrain's own surface. False, with the reason, if it does not fit.
bool create_soil(Soil *soil, const Terrain *terrain, const Sim_Config *config,
                 Linear_Allocator *allocator, char *error, u32 error_size);

// Moves one sample, keeping the block bounds and tile versions up to date.
void set_soil_height(Soil *soil, u32 row, u32 col, f32 height);

// True if (x, y) is on the soil rather than the rigid terrain.
bool is_on_soil(const Soil *soil, f32 x, f32 y);

// Whether terrain cell (row, col) belongs to the soil.
bool is_soil_cell(const Soil *soil, u32 row, u32 col);

// The ground's height at (x, y): the soil's where there is soil (soil may be NULL), else the
// terrain's.
f32 get_ground_height(const Terrain *terrain, const Soil *soil, f32 x, f32 y);
