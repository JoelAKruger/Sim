#include "core/soil.h"

#include <math.h>
#include <stdio.h>

#define SOIL_MAX_SAMPLES (16u * 1024u * 1024u)

bool create_soil(Soil *soil, const Terrain *terrain, const Sim_Config *config,
                 Linear_Allocator *allocator, char *error, u32 error_size)
{
    *soil = {};
    // A whole number of soil cells to a terrain cell, and an even number of terrain cells
    // each way, so the patch has a sample at its centre.
    u32 per_cell = max((u32)lroundf(terrain->spacing / config->soil_spacing), 1u);
    f32 spacing = terrain->spacing / (f32)per_cell;
    soil->cell_cols = 2 * max((u32)lroundf(0.5f * config->soil_size[0] / terrain->spacing), 1u);
    soil->cell_rows = 2 * max((u32)lroundf(0.5f * config->soil_size[1] / terrain->spacing), 1u);
    i32 center_col = (i32)lroundf((config->robot_spawn.x - terrain->origin_x) / terrain->spacing);
    i32 center_row = (i32)lroundf((terrain->origin_y - config->robot_spawn.y) / terrain->spacing);
    i32 first_col = center_col - (i32)soil->cell_cols / 2;
    i32 first_row = center_row - (i32)soil->cell_rows / 2;
    if (first_col < 0 || first_row < 0 || first_col + soil->cell_cols > terrain->cols - 1 ||
        first_row + soil->cell_rows > terrain->rows - 1) {
        snprintf(error, error_size,
                 "soil: a %.1f x %.1f m patch around robot.spawn doesn't fit on the terrain",
                 (f64)((f32)soil->cell_cols * terrain->spacing),
                 (f64)((f32)soil->cell_rows * terrain->spacing));
        return false;
    }
    soil->first_col = (u32)first_col;
    soil->first_row = (u32)first_row;

    Terrain *grid = &soil->grid;
    grid->cols = soil->cell_cols * per_cell + 1;
    grid->rows = soil->cell_rows * per_cell + 1;
    if ((u64)grid->cols * grid->rows > SOIL_MAX_SAMPLES) {
        snprintf(error, error_size,
                 "soil: %ux%u samples is too many; raise soil.spacing_m or shrink soil.size_m",
                 grid->cols, grid->rows);
        return false;
    }
    grid->spacing = spacing;
    grid->origin_x = terrain->origin_x + (f32)soil->first_col * terrain->spacing;
    grid->origin_y = terrain->origin_y - (f32)soil->first_row * terrain->spacing;
    grid->heights = ALLOCATE_ARRAY(allocator, f32, (u64)grid->rows * grid->cols);
    soil->block_cols = (grid->cols - 1 + SOIL_BLOCK_CELLS - 1) / SOIL_BLOCK_CELLS;
    soil->block_rows = (grid->rows - 1 + SOIL_BLOCK_CELLS - 1) / SOIL_BLOCK_CELLS;
    soil->block_low = ALLOCATE_ARRAY(allocator, f32, soil->block_rows * soil->block_cols);
    soil->block_high = ALLOCATE_ARRAY(allocator, f32, soil->block_rows * soil->block_cols);
    soil->tile_cols = (grid->cols - 1 + SOIL_TILE_CELLS - 1) / SOIL_TILE_CELLS;
    soil->tile_rows = (grid->rows - 1 + SOIL_TILE_CELLS - 1) / SOIL_TILE_CELLS;
    soil->tile_versions = ALLOCATE_ARRAY(allocator, u32, soil->tile_rows * soil->tile_cols);
    if (!grid->heights || !soil->block_low || !soil->block_high || !soil->tile_versions) {
        snprintf(error, error_size, "soil: out of memory for %ux%u samples", grid->cols,
                 grid->rows);
        return false;
    }
    for (u32 i = 0; i < soil->block_rows * soil->block_cols; i++) {
        soil->block_low[i] = INFINITY;
        soil->block_high[i] = -INFINITY;
    }
    for (u32 i = 0; i < soil->tile_rows * soil->tile_cols; i++) {
        soil->tile_versions[i] = 0;
    }

    // The terrain's surface, sampled on the terrain's own triangles.
    grid->min_height = INFINITY;
    grid->max_height = -INFINITY;
    for (u32 row = 0; row < grid->rows; row++) {
        for (u32 col = 0; col < grid->cols; col++) {
            f32 x = grid->origin_x + (f32)col * spacing;
            f32 y = grid->origin_y - (f32)row * spacing;
            set_soil_height(soil, row, col, get_terrain_height(terrain, x, y));
        }
    }
    soil->changes = 0;
    for (u32 i = 0; i < soil->tile_rows * soil->tile_cols; i++) {
        soil->tile_versions[i] = 0;
    }
    log_info("soil: %.1f x %.1f m of deformable soil, %ux%u samples %.0f mm apart",
             (f64)((f32)(grid->cols - 1) * spacing), (f64)((f32)(grid->rows - 1) * spacing),
             grid->cols, grid->rows, (f64)(spacing * 1000.0f));
    return true;
}

// The cells with sample index as a corner are cells index - 1 and index (along one axis of
// count samples). Gives the first and last block, of block_cells cells, holding either.
static void get_cell_range(u32 index, u32 count, u32 block_cells, u32 *first, u32 *last)
{
    u32 low = index > 0 ? index - 1 : 0;
    u32 high = min(index, count - 2);
    *first = low / block_cells;
    *last = high / block_cells;
}

void set_soil_height(Soil *soil, u32 row, u32 col, f32 height)
{
    Terrain *grid = &soil->grid;
    grid->heights[row * grid->cols + col] = height;
    grid->min_height = min(grid->min_height, height);
    grid->max_height = max(grid->max_height, height);
    u32 first_row, last_row, first_col, last_col;
    get_cell_range(row, grid->rows, SOIL_BLOCK_CELLS, &first_row, &last_row);
    get_cell_range(col, grid->cols, SOIL_BLOCK_CELLS, &first_col, &last_col);
    // Bounds only ever widen: a little loose once soil has moved, never wrong.
    for (u32 r = first_row; r <= last_row; r++) {
        for (u32 c = first_col; c <= last_col; c++) {
            u32 block = r * soil->block_cols + c;
            soil->block_low[block] = min(soil->block_low[block], height);
            soil->block_high[block] = max(soil->block_high[block], height);
        }
    }
    get_cell_range(row, grid->rows, SOIL_TILE_CELLS, &first_row, &last_row);
    get_cell_range(col, grid->cols, SOIL_TILE_CELLS, &first_col, &last_col);
    for (u32 r = first_row; r <= last_row; r++) {
        for (u32 c = first_col; c <= last_col; c++) {
            soil->tile_versions[r * soil->tile_cols + c]++;
        }
    }
    soil->changes++;
}

bool is_on_soil(const Soil *soil, f32 x, f32 y)
{
    const Terrain *grid = &soil->grid;
    f32 east = grid->origin_x + (f32)(grid->cols - 1) * grid->spacing;
    f32 south = grid->origin_y - (f32)(grid->rows - 1) * grid->spacing;
    return x >= grid->origin_x && x <= east && y <= grid->origin_y && y >= south;
}

bool is_soil_cell(const Soil *soil, u32 row, u32 col)
{
    return row >= soil->first_row && row < soil->first_row + soil->cell_rows &&
           col >= soil->first_col && col < soil->first_col + soil->cell_cols;
}

f32 get_ground_height(const Terrain *terrain, const Soil *soil, f32 x, f32 y)
{
    if (soil && is_on_soil(soil, x, y)) {
        return get_terrain_height(&soil->grid, x, y);
    }
    return get_terrain_height(terrain, x, y);
}
