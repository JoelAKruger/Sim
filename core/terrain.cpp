#include "core/terrain.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

static u32 hash_u32(u32 x)
{
    x ^= x >> 16;
    x *= 0x7feb352du;
    x ^= x >> 15;
    x *= 0x846ca68bu;
    x ^= x >> 16;
    return x;
}

// Uniform in [0, 1).
static f32 get_random_f32(u32 *state)
{
    *state = hash_u32(*state + 0x9e3779b9u);
    return (f32)(*state >> 8) * (1.0f / 16777216.0f);
}

// Value noise: random heights on an integer lattice, blended with a smoothstep.
static f32 get_lattice_value(i32 x, i32 y, u32 seed)
{
    u32 hash = hash_u32((u32)x * 0x8da6b343u ^ (u32)y * 0xd8163841u ^ seed * 0xcb1ab31fu);
    return (f32)(hash >> 8) * (2.0f / 16777216.0f) - 1.0f;
}

static f32 get_value_noise(f32 x, f32 y, u32 seed)
{
    f32 fx = floorf(x);
    f32 fy = floorf(y);
    i32 ix = (i32)fx;
    i32 iy = (i32)fy;
    f32 tx = x - fx;
    f32 ty = y - fy;
    tx = tx * tx * (3.0f - 2.0f * tx);
    ty = ty * ty * (3.0f - 2.0f * ty);
    f32 v00 = get_lattice_value(ix, iy, seed);
    f32 v10 = get_lattice_value(ix + 1, iy, seed);
    f32 v01 = get_lattice_value(ix, iy + 1, seed);
    f32 v11 = get_lattice_value(ix + 1, iy + 1, seed);
    f32 bottom = v00 + (v10 - v00) * tx;
    f32 top = v01 + (v11 - v01) * tx;
    return bottom + (top - bottom) * ty;
}

// Five octaves, each half the size and half the amplitude of the last. Roughly [-1, 1].
static f32 get_fractal_noise(f32 x, f32 y, u32 seed)
{
    f32 sum = 0.0f;
    f32 amplitude = 0.5f;
    for (u32 octave = 0; octave < 5; octave++) {
        sum += amplitude * get_value_noise(x, y, seed + octave * 101u);
        x *= 2.0f;
        y *= 2.0f;
        amplitude *= 0.5f;
    }
    return sum * 1.9f;
}

// A simple crater: a parabolic bowl 0.2 radii deep with a raised rim that falls
// away outside. d is distance from the centre in radii.
static f32 get_crater_profile(f32 d, f32 radius)
{
    f32 depth = 0.2f * radius;
    f32 rim = 0.05f * radius;
    f32 bowl = d < 1.0f ? depth * (d * d - 1.0f) : 0.0f;
    f32 rim_offset = (d - 1.0f) / 0.3f;
    return bowl + rim * expf(-rim_offset * rim_offset);
}

bool generate_terrain(Terrain *terrain, Linear_Allocator *allocator, const Sim_Config *config)
{
    *terrain = {};
    terrain->spacing = config->terrain_spacing;
    terrain->cols = (u32)(config->terrain_size[0] / config->terrain_spacing) + 1;
    terrain->rows = (u32)(config->terrain_size[1] / config->terrain_spacing) + 1;
    terrain->origin_x = -0.5f * (f32)(terrain->cols - 1) * terrain->spacing;
    terrain->origin_y = 0.5f * (f32)(terrain->rows - 1) * terrain->spacing;
    terrain->heights = ALLOCATE_ARRAY(allocator, f32, terrain->rows * terrain->cols);
    if (!terrain->heights) {
        return false;
    }

    struct Crater {
        f32 x, y, radius;
    };
    Crater craters[512] = {}; // skipped entries stay zero-radius
    u32 crater_count = min(config->crater_count, (u32)ARRAY_COUNT(craters));
    u32 random = config->terrain_seed;
    f32 half_x = 0.5f * config->terrain_size[0];
    f32 half_y = 0.5f * config->terrain_size[1];
    for (u32 i = 0; i < crater_count; i++) {
        // Mostly small craters, a few large ones: radius from 1 to 15 m, biased low.
        f32 t = get_random_f32(&random);
        Crater crater = {
            .x = (get_random_f32(&random) * 2.0f - 1.0f) * half_x,
            .y = (get_random_f32(&random) * 2.0f - 1.0f) * half_y,
            .radius = 1.0f + 14.0f * t * t * t,
        };
        f32 clear = crater.radius * 1.6f + 6.0f;
        if (crater.x * crater.x + crater.y * crater.y < clear * clear) {
            continue; // keep the spawn area clear
        }
        craters[i] = crater;
    }

    f32 hill_scale = 1.0f / 40.0f; // hills roughly 40 m across
    terrain->min_height = INFINITY;
    terrain->max_height = -INFINITY;
    for (u32 row = 0; row < terrain->rows; row++) {
        for (u32 col = 0; col < terrain->cols; col++) {
            f32 x = terrain->origin_x + (f32)col * terrain->spacing;
            f32 y = terrain->origin_y - (f32)row * terrain->spacing;
            f32 height = config->terrain_relief *
                         get_fractal_noise(x * hill_scale, y * hill_scale, config->terrain_seed);
            for (u32 i = 0; i < crater_count; i++) {
                const Crater *crater = &craters[i];
                if (crater->radius == 0.0f) {
                    continue;
                }
                f32 dx = x - crater->x;
                f32 dy = y - crater->y;
                f32 reach = crater->radius * 2.0f;
                if (dx * dx + dy * dy < reach * reach) {
                    height += get_crater_profile(sqrtf(dx * dx + dy * dy) / crater->radius,
                                                 crater->radius);
                }
            }
            terrain->heights[row * terrain->cols + col] = height;
            terrain->min_height = min(terrain->min_height, height);
            terrain->max_height = max(terrain->max_height, height);
        }
    }
    return true;
}

f32 get_terrain_height(const Terrain *terrain, f32 x, f32 y)
{
    f32 col_f = (x - terrain->origin_x) / terrain->spacing;
    f32 row_f = (terrain->origin_y - y) / terrain->spacing;
    col_f = min(max(col_f, 0.0f), (f32)(terrain->cols - 1));
    row_f = min(max(row_f, 0.0f), (f32)(terrain->rows - 1));
    u32 col = min((u32)col_f, terrain->cols - 2);
    u32 row = min((u32)row_f, terrain->rows - 2);
    f32 u = col_f - (f32)col;
    f32 v = row_f - (f32)row;

    // Each cell is split along the diagonal from (row + 1, col) to (row, col + 1).
    const f32 *h = terrain->heights;
    f32 h11 = h[row * terrain->cols + col];
    f32 h12 = h[row * terrain->cols + col + 1];
    f32 h21 = h[(row + 1) * terrain->cols + col];
    f32 h22 = h[(row + 1) * terrain->cols + col + 1];
    if (u + v <= 1.0f) {
        return h11 + u * (h12 - h11) + v * (h21 - h11);
    }
    return h22 + (1.0f - u) * (h21 - h22) + (1.0f - v) * (h12 - h22);
}

// Skips whitespace and '#' comments in a PNM header, then reads one decimal number.
static bool read_pnm_number(const u8 *data, u64 size, u64 *at, u32 *value)
{
    while (*at < size) {
        u8 c = data[*at];
        if (c == '#') {
            while (*at < size && data[*at] != '\n') {
                (*at)++;
            }
        } else if (c == ' ' || c == '\t' || c == '\r' || c == '\n') {
            (*at)++;
        } else {
            break;
        }
    }
    u64 start = *at;
    u64 number = 0;
    while (*at < size && data[*at] >= '0' && data[*at] <= '9' && number <= UINT32_MAX) {
        number = number * 10 + (data[*at] - '0');
        (*at)++;
    }
    *value = (u32)number;
    return *at > start && number <= UINT32_MAX;
}

bool load_heightmap(Terrain *terrain, Linear_Allocator *allocator, const Sim_Config *config,
                    char *error, u32 error_size)
{
    *terrain = {};
    const char *path = config->terrain_heightmap;
    u64 size = 0;
    u8 *data = (u8 *)read_file(path, &size);
    if (!data) {
        snprintf(error, error_size, "cannot read heightmap %s", path);
        return false;
    }
    u64 at = 2;
    u32 cols = 0, rows = 0, maxval = 0;
    bool header =
        size > 2 && data[0] == 'P' && data[1] == '5' && read_pnm_number(data, size, &at, &cols) &&
        read_pnm_number(data, size, &at, &rows) && read_pnm_number(data, size, &at, &maxval);
    at++; // exactly one whitespace byte separates the header from the pixels
    u32 bytes_per_sample = maxval > 255 ? 2 : 1;
    if (!header || maxval == 0 || maxval > 65535) {
        snprintf(error, error_size, "%s: not a binary PGM (P5) heightmap", path);
    } else if (cols < 2 || rows < 2 || cols > 16385 || rows > 16385) {
        snprintf(error, error_size, "%s: %ux%u pixels; 2 to 16385 per side are supported", path,
                 cols, rows);
    } else if (at + (u64)cols * rows * bytes_per_sample > size) {
        snprintf(error, error_size, "%s: truncated (%ux%u pixels expected)", path, cols, rows);
    } else {
        terrain->heights = ALLOCATE_ARRAY(allocator, f32, (u64)rows * cols);
        if (!terrain->heights) {
            snprintf(error, error_size, "%s: out of memory for %ux%u samples", path, cols, rows);
        }
    }
    if (!terrain->heights) {
        free(data);
        return false;
    }
    if (bytes_per_sample == 1) {
        log_warning(
            "terrain: %s has 8-bit samples (%.3f m steps); the terraces will make wheels chatter. "
            "Prefer a 16-bit PGM",
            path, (f64)(config->terrain_height / (f32)maxval));
    }

    terrain->rows = rows;
    terrain->cols = cols;
    terrain->spacing = config->terrain_spacing;
    terrain->origin_x = -0.5f * (f32)(cols - 1) * terrain->spacing;
    terrain->origin_y = 0.5f * (f32)(rows - 1) * terrain->spacing;
    terrain->min_height = INFINITY;
    terrain->max_height = -INFINITY;
    f32 scale = config->terrain_height / (f32)maxval;
    const u8 *pixels = data + at;
    for (u64 i = 0; i < (u64)rows * cols; i++) {
        // 16-bit PGM samples are big-endian.
        u32 sample =
            bytes_per_sample == 2 ? (u32)pixels[i * 2] << 8 | pixels[i * 2 + 1] : pixels[i];
        f32 height = (f32)min(sample, maxval) * scale;
        terrain->heights[i] = height;
        terrain->min_height = min(terrain->min_height, height);
        terrain->max_height = max(terrain->max_height, height);
    }
    free(data);
    log_info("terrain: %s, %ux%u samples, %.0f x %.0f m, heights %.2f to %.2f m", path, cols, rows,
             (f64)((f32)(cols - 1) * terrain->spacing), (f64)((f32)(rows - 1) * terrain->spacing),
             (f64)terrain->min_height, (f64)terrain->max_height);
    return true;
}
