#pragma once

#include <box3d/box3d.h>

#include "core/allocator.h"
#include "core/config.h"
#include "core/terrain.h"

// Boulders scattered over the terrain, fixed in the ground: irregular convex rocks, mostly
// small with a few large, partly buried. They are static Box3D hulls, so the robot, props
// and LiDAR all meet them. The layout comes from terrain.seed, so it is the same every run,
// and the area around the spawn point is kept clear.

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
    b3BodyId body; // one static body holding every rock's hull
    // Every rock's hull faces as world-space triangles with flat normals, for drawing, so
    // the renderer never needs Box3D.
    v3 *vertices;
    v3 *normals;
    u32 vertex_count;
};

bool create_boulders(Boulder_Field *field, b3WorldId world, const Terrain *terrain,
                     const Sim_Config *config, Linear_Allocator *allocator);
