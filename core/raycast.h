#pragma once

#include "core/allocator.h"
#include "core/math.h"
#include "core/soil.h"
#include "core/terrain.h"

// Ray casts for the sensors. They are done here, not by the physics engine, so that many
// threads can cast at once and the results are the same every run: against the ground (the
// terrain, and the soil set into it) and a list of convex solids, each riding on a pose
// that the world keeps up to date (a robot body's, a prop's) or fixed in the world.

enum Solid_Kind {
    SOLID_BOX,
    SOLID_SPHERE,
    SOLID_CYLINDER,
    SOLID_HULL,
};

// Points p with dot(normal, p) <= offset are inside.
struct Plane {
    v3 normal;
    f32 offset;
};

struct Solid {
    Solid_Kind kind;
    const Pose *frame; // the pose it rides on, or NULL for the world
    Pose local; // the solid in that frame
    v3 size; // box: half extents; sphere: radius (x); cylinder: radius (x), half length (z)
    u32 first_plane; // hull: its planes in Ray_Scene::planes, in the local frame
    u32 plane_count;
    f32 bound; // m, radius of a sphere about local.p that holds the solid
};

struct Ray_Scene {
    const Terrain *terrain;
    const Soil *soil; // or NULL
    Solid *solids;
    u32 solid_count;
    u32 solid_capacity;
    Plane *planes;
    u32 plane_count;
    u32 plane_capacity;
};

struct Ray_Hit {
    f32 distance; // m, or -1 for a miss
    v3 normal; // of the surface hit (either side)
};

bool create_ray_scene(Ray_Scene *scene, const Terrain *terrain, const Soil *soil,
                      u32 solid_capacity, u32 plane_capacity, Linear_Allocator *allocator);

// Solids, placed by local in frame (NULL for the world). False if the scene is full.
bool add_ray_box(Ray_Scene *scene, const Pose *frame, Pose local, v3 half_extents);
bool add_ray_sphere(Ray_Scene *scene, const Pose *frame, v3 center, f32 radius);
bool add_ray_cylinder(Ray_Scene *scene, const Pose *frame, Pose local, f32 radius, f32 length);
// A convex hull given as its triangles (three vertices each) in frame's coordinates.
bool add_ray_hull(Ray_Scene *scene, const Pose *frame, const v3 *triangles, u32 vertex_count);

// Where a solid's bounding sphere is now.
v3 get_solid_center(const Solid *solid);

// The first hit along a unit direction within max_distance. Only the listed solids are
// tried (all of them when solids is NULL). A ray that starts inside a solid doesn't hit it.
Ray_Hit cast_scene_ray(const Ray_Scene *scene, v3 origin, v3 direction, f32 max_distance,
                       const u32 *solids, u32 solid_count);
