#pragma once

#include "core/allocator.h"
#include "core/config.h"
#include "core/physics.h"
#include "core/raycast.h"
#include "core/soil.h"
#include "core/terrain.h"

// Boulders scattered over the terrain, fixed in the ground: irregular convex rocks, mostly
// small with a few large, partly buried. They are fixed convex hulls in the physics and in
// the ray scene, so the robot, props and LiDAR all meet them. The layout comes from
// terrain.seed, so it is the same every run. The area around the spawn point and the soil
// are kept clear.

#define BOULDER_SPAWN_CLEARANCE 6.0f // m around robot.spawn with no boulders

struct Boulder {
    v3 center;
    f32 radius; // m, of a sphere that holds the whole rock
    u32 first_vertex; // its triangles in Boulder_Field::vertices
    u32 vertex_count;
};

struct Boulder_Field {
    Boulder *boulders;
    u32 count;
    // Every rock's hull faces as world-space triangles with flat normals, for drawing.
    v3 *vertices;
    v3 *normals;
    u32 vertex_count;
};

// soil may be NULL.
bool create_boulders(Boulder_Field *field, Physics *physics, Ray_Scene *scene,
                     const Terrain *terrain, const Soil *soil, const Sim_Config *config,
                     Linear_Allocator *allocator);
