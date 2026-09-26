#pragma once

#include <raylib.h>

#include "core/terrain.h"

// How the world is lit: one hard, low sun (the Moon has no atmosphere) with shadows, a faint
// fill so shaded sides stay readable, procedural detail on the ground and rocks, a starfield
// behind everything, and the terrain fading to black at its edges.
//
// Everything that costs anything scales with the quality level:
//   0 low     no shadows, no surface detail
//   1 medium  2048² shadow map, 3×3 filtering; the robot casts its collision shapes' shadow
//   2 high    4096² shadow map, 5×5 filtering; the robot casts its full meshes' shadow

#define SHADING_STARS 1500

enum Shading_Surface {
    SURFACE_PLAIN, // vertex and material colour only (robot, props)
    SURFACE_REGOLITH, // the ground: fine grain, patches, darker slopes
    SURFACE_ROCK, // boulders: coarser grain
};

struct Shading {
    Shader lit;
    i32 sun_direction_location;
    i32 light_matrix_location;
    i32 shadow_map_location;
    i32 shadow_filter_location; // 0 off, 1 = 3×3, 2 = 5×5
    i32 shadow_texel_location;
    i32 surface_location;
    i32 detail_location;
    i32 terrain_bounds_location;
    i32 edge_fade_location;

    Shader caster; // depth only, for the shadow pass
    Material caster_material;
    u32 shadow_framebuffer;
    u32 shadow_depth; // depth texture
    i32 shadow_size; // pixels per side; 0 when shadows are off
    f32 shadow_extent; // m, width of the square the shadow map covers
    Matrix light_matrix; // world to shadow-map clip space

    Vector3 sun_direction; // the way the light travels (unit)
    Vector3 stars[SHADING_STARS]; // unit directions
    f32 star_brightness[SHADING_STARS];
    u32 quality;
};

// Needs the window. terrain gives the bounds the edge fade works from.
void create_shading(Shading *shading, const Terrain *terrain, u32 quality);
void destroy_shading(Shading *shading);

// Changes the quality level (0..2), rebuilding the shadow map to suit.
void set_shading_quality(Shading *shading, u32 quality);

// Sets which kind of surface the next lit draws are.
void set_shading_surface(const Shading *shading, Shading_Surface surface);

// The shadow pass: call before drawing the scene with the lit shader, outside any
// BeginMode3D. It covers a square around focus (normally the robot), and draw_casters draws
// the scene with the given depth-only material.
typedef void Shadow_Caster_Function(void *context, Material *material);
void render_shadow_map(Shading *shading, Vector3 focus, Shadow_Caster_Function *draw_casters,
                       void *context);

// Around drawing with the lit shader: binds the shadow map.
void begin_lit_drawing(const Shading *shading);
void end_lit_drawing(void);

// The starfield, in 2D behind the 3D view (call after clearing, before the 3D pass).
void draw_stars(const Shading *shading, Camera3D camera, Rectangle view);
