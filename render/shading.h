#pragma once

#include <raylib.h>

#include "core/terrain.h"

// How the world is lit: one hard, low sun (the Moon has no atmosphere) with shadows, a faint
// fill so shaded sides stay readable, procedural detail on the ground and rocks, a starfield
// behind everything, and the terrain fading to black at its edges.
//
// Shadows are cascaded: four maps in one square atlas, each covering a different area from
// the sun's point of view, and each fragment uses the finest one that covers it:
//   0  12 m around the robot: crisp shadows on and around it, wherever the view is
//   1  the near part of the view (out to 0.8 × the orbit distance)
//   2  the rest of the view, out to 3 × the orbit distance
//   3  100 m around the robot, so the sensor camera always has shadows
// Each is fitted to a fixed-size square that is snapped to its texels, so shadow edges
// don't shimmer as the robot or the view moves. Lookups are bilinear PCF, so they are
// smooth rather than blocky.
//
// Everything that costs anything scales with the quality level:
//   0 low     no shadows, no surface detail
//   1 medium  2048² atlas (1024² cascades), 2×2 filter taps; the robot casts its collision
//             shapes' shadow
//   2 high    4096² atlas (2048² cascades), 3×3 filter taps; the robot casts its full
//             meshes' shadow

#define SHADING_STARS 1500
#define SHADOW_CASCADES 4

// One cascade: the square (seen from the sun) its map covers, and its projection.
struct Shadow_Cascade {
    Vector3 center;
    f32 radius; // m: the square is 2 × radius wide
    Matrix light_matrix; // world to this cascade's clip space
};

enum Shading_Surface {
    SURFACE_PLAIN, // vertex and material colour only (robot, props)
    SURFACE_REGOLITH, // the ground: fine grain, patches, darker slopes
    SURFACE_ROCK, // boulders: coarser grain
};

struct Shading {
    Shader lit;
    i32 sun_direction_location;
    i32 light_matrices_location;
    i32 cascade_texel_location;
    i32 shadow_map_location;
    i32 shadow_filter_location; // 0 off, 1 = 2×2 bilinear taps, 2 = 3×3
    i32 surface_location;
    i32 detail_location;
    i32 terrain_bounds_location;
    i32 edge_fade_location;

    Shader caster; // depth only, for the shadow pass
    Material caster_material;
    u32 shadow_framebuffer;
    u32 shadow_depth; // depth texture
    i32 shadow_size; // atlas pixels per side (2 × 2 cascades); 0 when shadows are off
    Shadow_Cascade cascades[SHADOW_CASCADES];

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
// BeginMode3D. view is the viewer's camera and aspect its width over height; focus is the
// robot (or the view target without one). draw_casters draws the scene with the given
// depth-only material, and may skip whatever lies further than radius from center across
// the sun's direction.
typedef void Shadow_Caster_Function(void *context, Material *material, Vector3 center, f32 radius);
void render_shadow_map(Shading *shading, Camera3D view, f32 aspect, Vector3 focus,
                       Shadow_Caster_Function *draw_casters, void *context);

// Around drawing with the lit shader: binds the shadow map.
void begin_lit_drawing(const Shading *shading);
void end_lit_drawing(void);

// The starfield, in 2D behind the 3D view (call after clearing, before the 3D pass).
void draw_stars(const Shading *shading, Camera3D camera, Rectangle view);
